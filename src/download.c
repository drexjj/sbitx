#include <glib.h>
#include <stdio.h>
#include "sdr_ui.h"

// Functions for running scripts to download things:
// so far just cty.dat

static gboolean download_check_idle(gpointer data)
{
	char *output = (char *)data;
	if (output) {
		write_console(STYLE_LOG, output);
		printf("%s", output);
		g_free(output);
	}
	return FALSE; // run once
}

static void *download_check_thread(void *arg)
{
	(void)arg;
	gchar *stdout_buf = NULL;
	gint exit_status = 0;
	GError *gerr = NULL;
	gboolean ok;

	/* Run the update script and capture its output */
	ok = g_spawn_command_line_sync("/home/pi/sbitx/update-cty.sh",
										&stdout_buf, NULL, &exit_status, &gerr);

	char output[256];
	output[0] = '\0';

	if (!ok) {
		snprintf(output, sizeof(output), "Failed to run update-cty.sh: %s\n",
				(gerr && gerr->message) ? gerr->message : "unknown error");
	} else {
		if (stdout_buf && *stdout_buf) {
			/* Copy stdout into our local buffer and trim trailing newlines */
			strncpy(output, stdout_buf, sizeof(output) - 1);
			output[sizeof(output) - 1] = '\0';
			size_t len = strlen(output);
			while (len > 0 && (output[len - 1] == '\n' || output[len - 1] == '\r')) {
				output[len - 1] = '\0';
				len--;
			}
		} else {
			snprintf(output, sizeof(output), "update-cty.sh exited with status %d\n", exit_status);
		}
	}

	/* Duplicate the output for use in the main thread and schedule an idle callback */
	gchar *out_dup = g_strdup(output ? output : "");
	if (out_dup)
		g_idle_add(download_check_idle, out_dup);

	if (stdout_buf)
		g_free(stdout_buf);
	if (gerr)
		g_error_free(gerr);

	return NULL;
}

void download_check()
{
	pthread_t tid;
	if (pthread_create(&tid, NULL, download_check_thread, NULL) == 0)
		pthread_detach(tid);
}
