#!/bin/sh
cd /home/pi/sbitx

if [ -f data/big-cty.url ]; then
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
    echo "error: download link for cty.dat not found" >&2
    exit 1
fi

if [ -f data/big-cty.url ]; then
    prev=$(sed -n '1p' data/big-cty.url 2>/dev/null || printf '')
else
    prev=''
fi

echo $link > data/big-cty.url

if [ "$link" = "$prev" ]; then
    echo "no cty.dat update: download link unchanged since `stat -c '%w' data/big-cty.url`" >&2
    exit 0
fi

wget "$link" -O /tmp/bigcty-latest.zip
unzip -o /tmp/bigcty-latest.zip -d data cty.dat
rm /tmp/bigcty-latest.zip

echo "cty.dat is the latest as of `stat -c '%w' data/cty.dat`"
