void webserver_start();
void webserver_poll();
void webserver_stop();
void web_update(char *message);
int is_remote_browser_active();
int is_localhost_connection_only();
int get_active_connection_ips(char *buffer, int buffer_size);
