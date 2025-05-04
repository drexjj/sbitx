// Based on https://mongoose.ws/tutorials/websocket-server/

#include "mongoose.h"
#include "webserver.h"
#include <pthread.h>
#include <math.h>
#include <string.h>
#include <complex.h>
#include <fftw3.h>
#include <wiringPi.h>
#include "sdr.h"
#include "sdr_ui.h"
#include "logbook.h"
#include "hist_disp.h"
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <arpa/inet.h>
#include <netinet/in.h>

// Function declarations for browser microphone handling
extern int browser_mic_input(int16_t *samples, int count);
extern int is_browser_mic_active();

// HTTP and HTTPS endpoints
static const char *s_http_addr = "0.0.0.0:8080";  // Plain address without protocol
static const char *s_https_addr = "0.0.0.0:8443";  // Plain address without protocol

static const char *s_ssl_cert_path = "/home/pi/sbitx/ssl/cert.pem";
static const char *s_ssl_key_path = "/home/pi/sbitx/ssl/key.pem";

// Global buffers for TLS certificate and key to prevent memory issues
static char *g_cert_buf = NULL;
static char *g_key_buf = NULL;
static size_t g_cert_len = 0;
static size_t g_key_len = 0;

static char s_web_root[1000];
static char session_cookie[100];
static int active_websocket_connections = 0; // Counter for active WebSocket connections
static int quit_webserver = 0; // Flag to signal webserver thread to stop
static pthread_t webserver_thread; // Thread handle for the webserver

// Define a structure to track WebSocket connections
#define MAX_WS_CONNECTIONS 10
#define WS_CONNECTION_TIMEOUT_MS 5000  // 5 seconds timeout

typedef struct {
    struct mg_connection *conn;  // Pointer to the connection
    int64_t last_active_time;    // Timestamp of last activity
    int active;                  // Whether this connection is active
    char ip_addr[50];           // IP address of the client
} ws_connection_t;

static ws_connection_t ws_connections[MAX_WS_CONNECTIONS] = {0};
static int64_t last_ping_time = 0;  // Time of last ping check
static struct mg_mgr mgr;  // Event manager

// Debug flag for webserver logging
static int webserver_debug_enabled = 0; // Set to 1 to enable verbose logging

// Helper function to read a file into a dynamically allocated buffer
// Returns NULL on error, caller must free the buffer.
static char *read_file(const char *path, size_t *len)
{
  FILE *fp = fopen(path, "rb");
  if (fp == NULL) {
    perror("fopen failed");
    return NULL;
  }
  fseek(fp, 0, SEEK_END);
  *len = (size_t)ftell(fp);
  fseek(fp, 0, SEEK_SET);
  char *buf = (char *)malloc(*len + 1);
  if (buf != NULL) {
    size_t read_len = fread(buf, 1, *len, fp);
    if (read_len != *len) {
      fprintf(stderr, "fread failed: read %zu, expected %zu\n", read_len, *len);
      free(buf);
      buf = NULL;
      *len = 0;
    } else {
      buf[*len] = '\0'; // Null-terminate
    }
  }
  fclose(fp);
  return buf;
}

// Read file content into a buffer
static char *read_file_content(const char *path, size_t *size) {
  FILE *fp;
  char *data = NULL;
  *size = 0;
  if ((fp = fopen(path, "rb")) != NULL) {
    fseek(fp, 0, SEEK_END);
    *size = (size_t) ftell(fp);
    fseek(fp, 0, SEEK_SET);
    data = (char *) malloc(*size + 1);
    if (data != NULL) {
      fread(data, 1, *size, fp);
      data[*size] = '\0';
    }
    fclose(fp);
  }
  return data;
}

static void web_respond(struct mg_connection *c, char *message){
	// Check if connection is still valid before sending
	if (c && !c->is_closing) {
		// Send the message
		mg_ws_send(c, message, strlen(message), WEBSOCKET_OP_TEXT);
	}
}

static void get_console(struct mg_connection *c){
	char buff[2100];
	
	int n = web_get_console(buff, 2000);
	if (!n)
		return;
	mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
}

static void get_updates(struct mg_connection *c, int all){
	//send the settings of all the fields to the client
	char buff[2000];
	int i = 0;

	get_console(c);

	while(1){
		int update = remote_update_field(i, buff);
		// return of -1 indicates the eof fields
		if (update == -1)
			return;
	//send the status anyway
		if (all || update )
			mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT); 
		i++;
	}
}

static void do_login(struct mg_connection *c, char *key){

	char passkey[20];
	get_field_value("#passkey", passkey);

	//look for key only on non-local ip addresses
	// Check if IP is 127.0.0.1 (localhost)
	if ((!key || strcmp(passkey, key)) && 
	    !(c->rem.ip[0] == 127 && c->rem.ip[1] == 0 && c->rem.ip[2] == 0 && c->rem.ip[3] == 1)){
		web_respond(c, "login error");
		c->is_draining = 1;
		printf("passkey didn't match. Closing socket\n");
		return;
	}
	
	hd_createGridList(); // oz7bx: Make the list up to date at the beginning of a session
	sprintf(session_cookie, "%x", rand());
	char response[100];
	sprintf(response, "login %s", session_cookie);
	web_respond(c, response);	
	get_updates(c, 1);
}

static int16_t remote_samples[10000]; //the max samples are set by the queue lenght in modems.c

static void get_spectrum(struct mg_connection *c){
	char buff[3000];
	web_get_spectrum(buff);
	mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
	get_updates(c, 0);
}

static void get_audio(struct mg_connection *c){
	char buff[3000];
	web_get_spectrum(buff);
	mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
	get_updates(c, 0);

	int count = remote_audio_output(remote_samples);		
	if (count > 0)
		mg_ws_send(c, remote_samples, count * sizeof(int16_t), WEBSOCKET_OP_BINARY);
}

static void get_logs(struct mg_connection *c, char *args){
	char logbook_path[200];
	char row_response[1000], row[1000];
	char query[100];
	int	row_id;

	query[0] = 0;
	row_id = atoi(strtok(args, " "));
	logbook_query(strtok(NULL, " \t\n"), row_id, logbook_path);
	FILE *pf = fopen(logbook_path, "r");
	if (!pf)
		return;
	while(fgets(row, sizeof(row), pf)){
		sprintf(row_response, "QSO %s", row);
		web_respond(c, row_response); 
	}
	fclose(pf);
}

void get_macros_list(struct mg_connection *c){
	char macros_list[2000], out[3000];
	macro_list(macros_list);
	sprintf(out, "macros_list %s", macros_list);
	web_respond(c, out);
}

void get_macro_labels(struct mg_connection *c){
	char key_list[2000], out[3000];
	macro_get_keys(key_list);
	sprintf(out, "macro_labels %s", key_list);
	web_respond(c, out);
}

char request[200];
int request_index = 0;

typedef struct {
  struct mg_tls_opts tls_opts;
  uint16_t https_port;
} webserver_data_t;

static void web_despatcher(struct mg_connection *c, struct mg_ws_message *wm){
	// Check if this is binary data (browser microphone audio)
	if (wm->data.len > 0 && wm->flags & 2) { 
		// Binary data flag
		// Process browser microphone data
		// Always accept browser mic data - the browser will only send when in TX mode
		// and the browser_mic_input function will handle the data appropriately
		int16_t *audio_samples = (int16_t *)wm->data.buf;
		int sample_count = wm->data.len / sizeof(int16_t);
		
		// Pass the browser microphone data to the audio processing chain
		browser_mic_input(audio_samples, sample_count);
		return;
	}

	// Handle text messages
	if (wm->data.len > 99)
		return;

	strncpy(request, wm->data.buf, wm->data.len);	
	request[wm->data.len] = 0;
	//handle the 'no-cookie' situation
	char *cookie = NULL;
	char *field = NULL;
	char *value = NULL;

	cookie = strtok(request, "\n");
	field = strtok(NULL, "=");
	value = strtok(NULL, "\n");

	if (field == NULL || cookie == NULL){
		printf("Invalid request on websocket\n");
		web_respond(c, "quit Invalid request on websocket");
		c->is_draining = 1;
	}
	else if (strlen(field) > 100 || strlen(field) <  2 || strlen(cookie) > 40 || strlen(cookie) < 4){
		printf("Ill formed request on websocket\n");
		web_respond(c, "quit Illformed request");
		c->is_draining = 1;
	}
	else if (!strcmp(field, "login")){
		printf("trying login with passkey : [%s]\n", value);
		do_login(c, value);
	}
	else if (cookie == NULL || strcmp(cookie, session_cookie)){
		web_respond(c, "quit expired");
		printf("Cookie not found, closing socket %s vs %s\n", cookie, session_cookie);
		c->is_draining = 1;
	}
	else if (!strcmp(field, "spectrum"))
		get_spectrum(c);
	else if (!strcmp(field, "audio"))
		get_audio(c);
	else if (!strcmp(field, "logbook"))
		get_logs(c, value);
	else if (!strcmp(field, "macros_list"))
		get_macros_list(c);
	else if (!strcmp(field, "refresh"))
		get_updates(c, 1);
	else{
		char buff[1200];
		if (value)
			sprintf(buff, "%s %s", field, value);
		else
			strcpy(buff, field);
		remote_execute(buff);
		get_updates(c, 0);
	}
}

static void fn(struct mg_connection *c, int ev, void *ev_data) {
  webserver_data_t *ws_data = (webserver_data_t *)c->mgr->userdata; // Get our data

  if (ev == MG_EV_ACCEPT) {
    // Log when a connection is accepted
    char addr[INET6_ADDRSTRLEN]; 
    int af = c->rem.is_ip6 ? AF_INET6 : AF_INET;
    uint16_t local_port = mg_ntohs(c->loc.port);
    inet_ntop(af, c->rem.ip, addr, sizeof(addr));

    if (ws_data != NULL && local_port == ws_data->https_port) {
      // Connection on HTTPS port
      if (webserver_debug_enabled) {
          printf("MG_EV_ACCEPT: HTTPS Conn from %s on port %d. Initializing TLS...\n", addr, local_port);
      }
      // Initialize TLS for this connection
      mg_tls_init(c, &ws_data->tls_opts);
    } else {
      // Connection on other port (assume HTTP)
      if (webserver_debug_enabled) {
          printf("MG_EV_ACCEPT: HTTP Conn from %s on port %d, is_tls: %d\n", addr, local_port, c->is_tls);
      }
    }
  } else if (ev == MG_EV_ERROR) {
    // Log errors only if debugging is enabled
    if (webserver_debug_enabled) {
        printf("MG_EV_ERROR: %s\n", (char *) ev_data);
    }
  } else if (ev == MG_EV_CLOSE) {
    // Optionally log connection close if debugging
    if (webserver_debug_enabled) {
        char addr[32];
        int af = c->rem.is_ip6 ? AF_INET6 : AF_INET;
        inet_ntop(af, c->rem.ip, addr, sizeof(addr));
        printf("MG_EV_CLOSE: Conn from %s\n", addr);
    }
    
    // Check if this was a WebSocket connection
    if (c->is_websocket) {
      // Remove from our connection tracking array
      for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
        if (ws_connections[i].active && ws_connections[i].conn == c) {
          ws_connections[i].active = 0;
          ws_connections[i].conn = NULL;
          break;
        }
      }
      
      active_websocket_connections--;
      if (active_websocket_connections < 0) active_websocket_connections = 0; // Safety check
      
      if (webserver_debug_enabled) {
        printf("WebSocket connection closed, active connections: %d\n", active_websocket_connections);
      }
      
      // Just update the connection status - the regular UI update cycle will handle the transition
      if (active_websocket_connections == 0) {
        // Send a simple refresh message
        web_update("refresh");
      }
    }
  } else if (ev == MG_EV_TLS_HS) {
    // Log TLS Handshake result only if debugging
    if (webserver_debug_enabled) {
        printf("MG_EV_TLS_HS: Handshake %s. TLS established: %d, Error: %s\n", 
           ev_data == NULL ? "SUCCESS" : "FAILED", 
           c->is_tls,
           ev_data ? (char *)ev_data : "(none)");
    }
  } else if (ev == MG_EV_HTTP_MSG) {
    struct mg_http_message *hm = (struct mg_http_message *) ev_data;
 // Determine if redirection should happen based on IP
    int redirect_http_to_https;
    if (c->rem.ip[0] == 127 && c->rem.ip[1] == 0 && c->rem.ip[2] == 0 && c->rem.ip[3] == 1) {
        // Connection is from localhost, disable redirect
        redirect_http_to_https = 0;
    } else {
        // Connection is not from localhost, enable redirect 
        redirect_http_to_https = 1;
    }
    // Check for HTTP->HTTPS redirect *before* other handling
    if (redirect_http_to_https && !c->is_tls) {
      // Construct the target URL: https://sbitx.local:8443 + original URI
      char https_url[2048];
      snprintf(https_url, sizeof(https_url), "https://sbitx.local:8443%.*s", 
               (int)hm->uri.len, hm->uri.buf);
      
      // Construct the Location header string, including Content-Length: 0
      char redir_headers[2100]; 
      snprintf(redir_headers, sizeof(redir_headers), "Location: %s\r\nContent-Length: 0\r\n", https_url);

      // Send 302 redirect using the extra_headers parameter (3rd arg), empty body format (4th arg)
      mg_http_reply(c, 302, redir_headers, ""); 
            
      // Stop processing this request after sending the redirect
      return; 
    }

    // Log basic HTTP message receipt only if debugging (if not redirected)
    if (webserver_debug_enabled) {
        printf("MG_EV_HTTP_MSG received on %s connection for URI %.*s\n", 
               c->is_tls ? "HTTPS" : "HTTP", (int)hm->uri.len, hm->uri.buf);
    }

    if (mg_match(hm->uri, mg_str("/websocket"), NULL)) {
      // Upgrade to websocket. From now on, a connection is a full-duplex
      // Websocket connection, which will receive MG_EV_WS_MSG events.
      // Upgrade to websocket for audio support
      mg_ws_upgrade(c, hm, NULL);
    } else if (mg_match(hm->uri, mg_str("/rest"), NULL)) {
      // Serve REST response
      mg_http_reply(c, 200, "", "{\"result\": %d}\n", 123);
    } else {
      // Serve static files
      struct mg_http_serve_opts opts = {.root_dir = s_web_root};
      mg_http_serve_dir(c, ev_data, &opts);
    }
  } else if (ev == MG_EV_WS_MSG) {
    // Got websocket frame. Received data is wm->data
    struct mg_ws_message *wm = (struct mg_ws_message *) ev_data;
    
    // Update the last active time for this connection
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
      if (ws_connections[i].active && ws_connections[i].conn == c) {
        ws_connections[i].last_active_time = mg_millis();
        break;
      }
    }
    
    // Handle pong messages
    if (wm->flags == WEBSOCKET_OP_PONG) {
      // Just update the timestamp, which we already did above
      if (webserver_debug_enabled) {
        printf("Received pong from client\n");
      }
    } else {
      // Regular message
      web_despatcher(c, wm);
    }
  } else if (ev == MG_EV_WS_OPEN) {
    // WebSocket connection opened
    active_websocket_connections++;
    
    // Add to our connection tracking array
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
      if (!ws_connections[i].active) {
        ws_connections[i].conn = c;
        ws_connections[i].last_active_time = mg_millis();
        ws_connections[i].active = 1;
        
        // Store the client IP address
        char ip_str[50];
        // Format IP address manually using the connection's remote address (without port)
        snprintf(ip_str, sizeof(ip_str), "%d.%d.%d.%d", 
                 c->rem.ip[0], c->rem.ip[1], c->rem.ip[2], c->rem.ip[3]);
        strncpy(ws_connections[i].ip_addr, ip_str, sizeof(ws_connections[i].ip_addr)-1);
        ws_connections[i].ip_addr[sizeof(ws_connections[i].ip_addr)-1] = '\0'; // Ensure null termination
        
        break;
      }
    }
    
    if (webserver_debug_enabled) {
      printf("WebSocket connection opened, active connections: %d\n", active_websocket_connections);
    }

  }
}

// Check for stale connections and send pings
void check_websocket_connections() {
  int64_t current_time = mg_millis();
  int connections_closed = 0;
  
  // Send pings every 2 seconds
  if (current_time - last_ping_time > 2000) {
    last_ping_time = current_time;
    
    // Check each connection
    for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
      if (ws_connections[i].active && ws_connections[i].conn != NULL) {
        // Check if connection has timed out
        if (current_time - ws_connections[i].last_active_time > WS_CONNECTION_TIMEOUT_MS) {
          // Connection timed out, mark as inactive
          if (webserver_debug_enabled) {
            printf("WebSocket connection timed out and closed\n");
          }
          
          // Close the connection safely
          struct mg_connection *conn = ws_connections[i].conn;
          if (conn && !conn->is_closing) {
            // Only try to send close frame if connection is still valid
            mg_ws_send(conn, "", 0, WEBSOCKET_OP_CLOSE);
          }
          
          // Mark as inactive regardless of close success
          ws_connections[i].active = 0;
          ws_connections[i].conn = NULL;
          connections_closed++;
        } else {
          // Send a ping to keep the connection alive
          // Only if connection is still valid
          struct mg_connection *conn = ws_connections[i].conn;
          if (conn && !conn->is_closing) {
            mg_ws_send(conn, "ping", 4, WEBSOCKET_OP_PING);
          }
        }
      }
    }
    
    // If we closed any connections, update the counter
    if (connections_closed > 0) {
      active_websocket_connections -= connections_closed;
      if (active_websocket_connections < 0) active_websocket_connections = 0;
      
      // If all connections are now closed, just update the connection status
      if (active_websocket_connections == 0) {
        // Send a simple refresh message
        web_update("refresh");
      }
    }
  }
}

void *webserver_thread_function(void *server){
  // Initialize global manager
  mg_mgr_init(&mgr);
  
  // Note: Mongoose version may not support mg_mgr_set_option
  // We'll handle buffer issues with careful connection management instead
  
  // Prepare webserver data (TLS opts and port) for event handler
  // Allocate on heap instead of stack to ensure it persists
  webserver_data_t *ws_data = (webserver_data_t *)calloc(1, sizeof(webserver_data_t));
  if (ws_data == NULL) {
    fprintf(stderr, "Failed to allocate memory for webserver data\n");
    mg_mgr_free(&mgr);
    return NULL;
  }
  
  uint16_t https_port_num = 0;

  // Parse HTTPS port from address string
  // Basic parsing: find last ':' and convert the rest to int
  const char *port_str = strrchr(s_https_addr, ':');
  if (port_str != NULL) {
      https_port_num = (uint16_t)atoi(port_str + 1);
  }

  // Free previous buffers if they exist (for potential server restart)
  if (g_cert_buf != NULL) {
    free(g_cert_buf);
    g_cert_buf = NULL;
  }
  if (g_key_buf != NULL) {
    free(g_key_buf);
    g_key_buf = NULL;
  }

  // Read certificate and key files into memory buffers
  g_cert_buf = read_file(s_ssl_cert_path, &g_cert_len);
  g_key_buf = read_file(s_ssl_key_path, &g_key_len);

  if (https_port_num > 0 && g_cert_buf != NULL && g_key_buf != NULL) {
      ws_data->https_port = https_port_num;
      ws_data->tls_opts.cert = mg_str_n(g_cert_buf, g_cert_len);
      ws_data->tls_opts.key = mg_str_n(g_key_buf, g_key_len);
      if (webserver_debug_enabled) {
          printf("TLS data prepared for port %d\n", ws_data->https_port);
      }
  } else {
      if (https_port_num == 0) fprintf(stderr, "Could not parse HTTPS port from %s\n", s_https_addr);
      if (g_cert_buf == NULL) fprintf(stderr, "Failed to read certificate file: %s\n", s_ssl_cert_path);
      if (g_key_buf == NULL) fprintf(stderr, "Failed to read key file: %s\n", s_ssl_key_path);
      fprintf(stderr, "HTTPS will not be enabled.\n");
  }

  // Set the user data pointer for the manager
  mgr.userdata = ws_data; 

  // Create HTTP listener - this will handle both HTTP and WebSocket connections
  if (webserver_debug_enabled) {
      printf("Starting HTTP listener on %s\n", s_http_addr);
  }
  if (mg_http_listen(&mgr, s_http_addr, fn, &mgr) == NULL) {
    fprintf(stderr, "Cannot listen on %s\n", s_http_addr);
    // Clean up resources
    free(g_cert_buf);
    free(g_key_buf);
    g_cert_buf = NULL;
    g_key_buf = NULL;
    free(ws_data);
    mg_mgr_free(&mgr);
    return NULL; // Exit thread if HTTP fails
  }

  // HTTPS Listener (using mg_http_listen)
  // Only attempt if TLS data was prepared successfully
  if (ws_data->https_port > 0 && ws_data->tls_opts.cert.len > 0) {
      if (webserver_debug_enabled) {
          printf("Starting HTTPS listener on %s\n", s_https_addr);
      }
      // Use mg_http_listen instead of mg_listen
      if (mg_http_listen(&mgr, s_https_addr, fn, &mgr) == NULL) {
          fprintf(stderr, "Cannot listen on %s\n", s_https_addr);
          // Non-fatal? Or should we abort?
          // For now, just print warning, HTTP might still work.
      }
  } else {
      if (webserver_debug_enabled) {
          printf("Skipping HTTPS listener setup due to missing cert/key/port.\n");
      }
  }

  // Start event loop
  if (webserver_debug_enabled) {
      printf("Webserver started.\n");
  }

  // Event loop
  while(!quit_webserver){
    mg_mgr_poll(&mgr, 100);  // Poll for 100ms
    
    // Check for stale connections
    check_websocket_connections();
  }

  // Cleanup (will be reached when quit_webserver is set)
  // First, close all active connections gracefully
  for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
    if (ws_connections[i].active && ws_connections[i].conn != NULL) {
      struct mg_connection *conn = ws_connections[i].conn;
      if (conn && !conn->is_closing) {
        mg_ws_send(conn, "", 0, WEBSOCKET_OP_CLOSE);
      }
      ws_connections[i].active = 0;
      ws_connections[i].conn = NULL;
    }
  }
  
  // Free resources
  free(g_cert_buf);
  free(g_key_buf);
  g_cert_buf = NULL;
  g_key_buf = NULL;
  free(ws_data);
  mg_mgr_free(&mgr);
  return NULL;
}

// Function to check if any remote browser sessions are active
int is_remote_browser_active() {
  return active_websocket_connections > 0;
}

// Function to get the IP addresses of active connections
// Returns a comma-separated list of IP addresses in the provided buffer
// Returns the number of active connections
int get_active_connection_ips(char *buffer, int buffer_size) {
  int count = 0;
  buffer[0] = '\0'; // Initialize empty string
  
  for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
    if (ws_connections[i].active) {
      // Add comma if not the first IP
      if (count > 0) {
        strncat(buffer, ", ", buffer_size - strlen(buffer) - 1);
      }
      
      // Add the IP address
      strncat(buffer, ws_connections[i].ip_addr, buffer_size - strlen(buffer) - 1);
      count++;
    }
  }
  
  return count;
}

void webserver_stop(){
	// Signal the thread to stop
	quit_webserver = 1;
	
	// Wait for the thread to finish (optional)
	pthread_join(webserver_thread, NULL);
	
	// Reset the flag for potential restart
	quit_webserver = 0;
}

// Function to send updates to all connected WebSocket clients
void web_update(char *message) {
	// Iterate through all active connections and send the message
	for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
		if (ws_connections[i].active && ws_connections[i].conn != NULL) {
			struct mg_connection *conn = ws_connections[i].conn;
			// Only send if connection is still valid
			if (conn && !conn->is_closing) {
				// Send the message, no exception handling in C
				mg_ws_send(conn, message, strlen(message), WEBSOCKET_OP_TEXT);
			}
		}
	}
}

// Webserver start function

void webserver_start(){
	char directory[200];	//dangerous, find the MAX_PATH and replace 200 with it

	//TODO:  Make a helper function for this path stuff - n1qm	
	//Get symlink that points to this executables
	int readPath = readlink("/proc/self/exe", directory, 200);
	
	//Find the last path seperator
	int lastSep = 0;
	for (int i=0;i < readPath;i++) {
		if (directory[i] == '/')
			lastSep=i;
	}

	//Trim string at last seperator if > 0
	if (lastSep > 0)
		directory[lastSep] = '\0';
	else
		directory[readPath]='\0';
	//directoryPath should now be where the sbitx binary lives

	//char *path = getenv("HOME");
	strcpy(s_web_root, directory);
	strcat(s_web_root, "/web");
	//printf("Dir %s\n",s_web_root);
	//logbook_open();
 	pthread_create( &webserver_thread, NULL, webserver_thread_function, 
		(void*)NULL);
}
