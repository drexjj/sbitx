#include <gtk/gtk.h>
#include <stdlib.h>
#include <unistd.h>

void regenerate_keys(GtkWidget *widget, gpointer label) {
    gtk_label_set_text(GTK_LABEL(label), "Regenerating SSH keys...");

    // Define the SSH key regeneration command string
    const char *commands[] = {
        "sudo systemctl stop ssh",
        "sudo rm -f /etc/ssh/ssh_host_*",
        "sudo ssh-keygen -t rsa -f /etc/ssh/ssh_host_rsa_key -N ''",
        "sudo ssh-keygen -t ecdsa -f /etc/ssh/ssh_host_ecdsa_key -N ''",
        "sudo ssh-keygen -t ed25519 -f /etc/ssh/ssh_host_ed25519_key -N ''",
        "sudo systemctl start ssh",
        NULL
    };

    for (int i = 0; commands[i] != NULL; i++) {
        int result = system(commands[i]);
        if (result != 0) {
            gtk_label_set_text(GTK_LABEL(label), "Error: Could not regenerate keys.");
            return;
        }
    }

    gtk_label_set_text(GTK_LABEL(label), "SSH key regeneration complete.");
}

int main(int argc, char *argv[]) {
    GtkWidget *window;
    GtkWidget *grid;
    GtkWidget *button;
    GtkWidget *label;

    gtk_init(&argc, &argv);

    // Main window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "SSH Key Regenerator");
    gtk_window_set_default_size(GTK_WINDOW(window), 325, 80);
    g_signal_connect(window, "destroy", G_CALLBACK(gtk_main_quit), NULL);

    grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(window), grid);

    label = gtk_label_new("Click the button to regenerate SSH keys.");
    gtk_grid_attach(GTK_GRID(grid), label, 0, 0, 1, 1);

    button = gtk_button_new_with_label("Regenerate SSH Keys");
    g_signal_connect(button, "clicked", G_CALLBACK(regenerate_keys), label);
    gtk_grid_attach(GTK_GRID(grid), button, 0, 1, 1, 1);

    gtk_widget_show_all(window);
    gtk_main();

    return 0;
}
