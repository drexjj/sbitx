#include <gtk/gtk.h>

GtkWidget *ftx_rules_ui(GtkWidget* parentWindow);

// a stub that won't be called
int extract_single_semantic(const char* text, int text_len, int span, char *out, int outlen)
{
	return 0;
}

int main(int argc, char *argv[])
{
	gtk_init(&argc, &argv);
	GtkWidget *dialog = ftx_rules_ui(NULL);
	g_signal_connect (dialog, "destroy", G_CALLBACK (gtk_main_quit), NULL);
	gtk_main();
	return 0;
}
