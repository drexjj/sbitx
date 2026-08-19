#!/bin/sh

sanitize_cty_file()
{
	# Keep valid semicolon-delimited CTY records and preserve corrupt input separately.
	input=$1
	output=$2
	marker=$3
	temp=$(mktemp "${output}.tmp.XXXXXX") || return 2
	count_file=$(mktemp "${output}.count.XXXXXX") || {
		rm -f "$temp"
		return 2
	}

	# CTY.DAT is ASCII text; allow record formatting whitespace but reject other
	# control or non-ASCII bytes before passing the record to the C parser.
	if ! LC_ALL=C awk -v RS=';' -v count_file="$count_file" '
		BEGIN { malformed = 0 }
		{
			if ($0 ~ /^[[:space:]]*$/)
				next
			copy = $0
			colons = gsub(/:/, ":", copy)
			gsub(/[\r\n\t]/, "", copy)
			if (colons == 8 && copy ~ /^[[:print:]]*$/)
				printf "%s;", $0
			else
				malformed++
		}
		END { print malformed > count_file }
	' "$input" >"$temp"; then
		rm -f "$temp" "$count_file"
		return 2
	fi

	# Read the awk result and reject anything that is not a numeric record count.
	malformed=$(cat "$count_file")
	rm -f "$count_file"
	case "$malformed" in
		''|*[!0-9]*)
			rm -f "$temp"
			return 2
		;;
	esac

	if [ "$malformed" -gt 0 ]; then
		# Save the untouched input before replacing the active file.
		marker_temp=$(mktemp "${marker}.tmp.XXXXXX") || {
			rm -f "$temp"
			return 2
		}
		if ! cp "$input" "$marker_temp" || ! mv "$marker_temp" "$marker"; then
			rm -f "$temp" "$marker_temp"
			return 2
		fi
	fi

	if ! mv "$temp" "$output"; then
		rm -f "$temp"
		return 2
	fi

	if [ "$malformed" -gt 0 ]; then
		echo "sanitized $input: discarded $malformed malformed record(s); original saved to $marker" >&2
		return 1
	fi
	return 0
}

if [ "${1:-}" = "--sanitize-only" ]; then
	# This mode lets tests exercise sanitization without contacting the network.
	if [ "$#" -ne 4 ]; then
		echo "usage: $0 --sanitize-only input output marker" >&2
		exit 2
	fi

	sanitize_cty_file "$2" "$3" "$4"
	status=$?
	case "$status" in
		0) echo "cty.dat is clean" ;;
		1) echo "cty.dat was sanitized" ;;
		*) echo "failed to sanitize cty.dat" >&2; exit 1 ;;
	esac
	exit 0
fi

cd /home/pi/sbitx || exit 1

# Repair an existing file before deciding whether an update is needed.
if [ -f data/cty.dat ]; then
	sanitize_cty_file data/cty.dat data/cty.dat data/cty.dat.invalid
	status=$?
	if [ "$status" -gt 1 ]; then
		echo "error: failed to sanitize data/cty.dat" >&2
		exit 1
	fi
fi

force_update=0
# A marker means the last known file was corrupt, so bypass update short-circuits.
[ -f data/cty.dat.invalid ] && force_update=1

if [ "$force_update" -eq 0 ] && [ -f data/big-cty.url ]; then
	file_date=$(stat -c '%y' data/big-cty.url 2>/dev/null | cut -d' ' -f1)
	today=$(date +%Y-%m-%d)
	if [ -n "$file_date" ] && [ "$file_date" = "$today" ]; then
		echo "no cty.dat update: already checked today ($today)" >&2
		exit 0
	fi
fi

html=$(wget -q -O - 'https://www.country-files.com/category/big-cty/') || html=''

link=$(printf '%s' "$html" | grep -Eo "https?://www\.country-files\.com/bigcty/download/[^\"'<> ]+" | sed 's/&amp;/\&/g' | head -n1)

if [ -z "$link" ]; then
	echo "error: cty.dat update link not found" >&2
	exit 1
fi

if [ -f data/big-cty.url ]; then
	prev=$(sed -n '1p' data/big-cty.url 2>/dev/null || printf '')
else
	prev=''
fi

if [ "$force_update" -eq 0 ] && [ "$link" = "$prev" ]; then
	echo "no cty.dat update: download link unchanged" >&2
	exit 0
fi

archive=$(mktemp /tmp/bigcty-latest.XXXXXX.zip) || exit 1
download_dir=$(mktemp -d /tmp/bigcty.XXXXXX) || {
	rm -f "$archive"
	exit 1
}
trap 'rm -rf "$download_dir" "$archive"' EXIT

# Sanitize the download before it can replace the active CTY database.
if ! wget "$link" -O "$archive" || ! unzip -q "$archive" -d "$download_dir" cty.dat; then
	echo "error: failed to download cty.dat" >&2
	exit 1
fi

sanitize_cty_file "$download_dir/cty.dat" "$download_dir/cty.dat.sanitized" data/cty.dat.invalid
status=$?
if [ "$status" -gt 1 ]; then
	echo "error: downloaded cty.dat could not be sanitized" >&2
	exit 1
fi

install_tmp=$(mktemp data/cty.dat.tmp.XXXXXX) || exit 1
# Rename within data/ atomically so readers see a complete old or new file.
if ! cp "$download_dir/cty.dat.sanitized" "$install_tmp" || ! mv "$install_tmp" data/cty.dat; then
	rm -f "$install_tmp"
	exit 1
fi
printf '%s\n' "$link" > data/big-cty.url
echo "cty.dat is the latest as of `stat -c '%w' data/cty.dat`"

if [ "$status" -eq 0 ]; then
	rm -f data/cty.dat.invalid
	echo "cty.dat is the latest clean version"
else
	echo "cty.dat was installed after sanitization; original saved to data/cty.dat.invalid"
fi
