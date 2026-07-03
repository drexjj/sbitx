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
#include <sys/socket.h>
#include <netdb.h>
#include "dynamic_content.h"

// Function declaration for S-meter
extern int calculate_s_meter(struct rx *r, double rx_gain);

// External variables for voltage and current readings from INA260 sensor
extern float voltage;
extern float current;
extern int has_ina260;

// Function declarations for browser microphone handling
extern int browser_mic_input(int16_t *samples, int count);
extern int is_browser_mic_active();

// VNC proxy connection structure
typedef struct {
    struct mg_connection *client;  // WebSocket client connection
    struct mg_connection *server;  // Connection to VNC server
    int vnc_port;                  // VNC server port
    int active;                    // Whether this proxy is active
} vnc_proxy_t;

#define MAX_VNC_PROXIES 10
static vnc_proxy_t vnc_proxies[MAX_VNC_PROXIES] = {0};

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
static int webserver_debug_enabled = 1; // Set to 1 to enable verbose logging

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

	// Send S-meter value 
	struct rx *current_rx = rx_list;
	double rx_gain = (double)get_rx_gain();
	int s_meter_value = calculate_s_meter(current_rx, rx_gain);
	int s_units = s_meter_value / 100;
	int additional_db = s_meter_value % 100;
	sprintf(buff, "SMETER %d %d", s_units, additional_db);
	mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);

	// Send zerobeat value for CW modes
	if (!strcmp(field_str("MODE"), "CW") || !strcmp(field_str("MODE"), "CWR")) {
		int zerobeat_value = calculate_zero_beat(current_rx, 96000.0);
		sprintf(buff, "ZEROBEAT %d", zerobeat_value);
		mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
	}
	
	// Send voltage and current readings if INA260 is equipped
	if (has_ina260 == 1) {
		sprintf(buff, "VOLTAGE %.2f", voltage);
		mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
		
		sprintf(buff, "CURRENT %.2f", current);
		mg_ws_send(c, buff, strlen(buff), WEBSOCKET_OP_TEXT);
	}

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
	logbook_query(strtok(NULL, " \t\n"), row_id, logbook_path, sizeof(logbook_path));
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

// Execute a shell script and return the result
// Handle VNC proxy WebSocket connections
static void handle_vnc_proxy(struct mg_connection *c, int ev, void *ev_data) {
    // Get proxy data from connection's user_data
    vnc_proxy_t *proxy = (vnc_proxy_t *)c->fn_data;
    
    if (ev == MG_EV_READ) {
        // Forward data from VNC server to WebSocket client
        if (proxy && proxy->client && !proxy->client->is_closing) {
            // Send data from VNC server to WebSocket client
            mg_ws_send(proxy->client, c->recv.buf, c->recv.len, WEBSOCKET_OP_BINARY);
            // Clear the receive buffer after forwarding
            mg_iobuf_del(&c->recv, 0, c->recv.len);
        }
    } else if (ev == MG_EV_CLOSE) {
        // VNC server connection closed
        if (proxy) {
            if (webserver_debug_enabled) {
                printf("VNC server connection closed\n");
            }
            
            // Mark this proxy as inactive
            proxy->active = 0;
            proxy->server = NULL;
            
            // Close the client connection if it's still open
            if (proxy->client && !proxy->client->is_closing) {
                proxy->client->is_closing = 1;
            }
        }
    } else if (ev == MG_EV_ERROR) {
        // Connection error
        if (webserver_debug_enabled) {
            printf("VNC proxy error: %s\n", (char *)ev_data);
        }
    }
}

// Create a new VNC proxy connection
static void create_vnc_proxy(struct mg_connection *c, int vnc_port) {
    // Find an available proxy slot
    int proxy_index = -1;
    for (int i = 0; i < MAX_VNC_PROXIES; i++) {
        if (!vnc_proxies[i].active) {
            proxy_index = i;
            break;
        }
    }
    
    if (proxy_index == -1) {
        // No available proxy slots
        if (webserver_debug_enabled) {
            printf("No available VNC proxy slots\n");
        }
        mg_http_reply(c, 500, "Content-Type: application/json\r\n", 
                     "{\"status\":\"error\",\"message\":\"No available VNC proxy slots\"}\n");
        return;
    }
    
    // Create a connection to the VNC server
    char addr[32];
    snprintf(addr, sizeof(addr), "127.0.0.1:%d", vnc_port);
    
    if (webserver_debug_enabled) {
        printf("Creating VNC proxy to %s\n", addr);
    }
    
    // Connect to the VNC server
    struct mg_connection *server_conn = mg_connect(c->mgr, addr, handle_vnc_proxy, &vnc_proxies[proxy_index]);
    // Note: The connection's fn_data will be set to &vnc_proxies[proxy_index] by mg_connect
    if (server_conn == NULL) {
        // Failed to connect to VNC server
        if (webserver_debug_enabled) {
            printf("Failed to connect to VNC server at %s\n", addr);
        }
        mg_http_reply(c, 500, "Content-Type: application/json\r\n", 
                     "{\"status\":\"error\",\"message\":\"Failed to connect to VNC server\"}\n");
        return;
    }
    
    // Store the connections in the proxy structure
    vnc_proxies[proxy_index].client = c;
    vnc_proxies[proxy_index].server = server_conn;
    vnc_proxies[proxy_index].vnc_port = vnc_port;
    vnc_proxies[proxy_index].active = 1;
    
    // Respond with success
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", 
                 "{\"status\":\"success\",\"message\":\"VNC proxy created\"}\n");
}

// Handle WebSocket messages for VNC proxy
static void handle_vnc_ws(struct mg_connection *c, struct mg_ws_message *wm) {
    // Find the proxy for this client connection
    for (int i = 0; i < MAX_VNC_PROXIES; i++) {
        if (vnc_proxies[i].active && vnc_proxies[i].client == c) {
            // Forward data from WebSocket client to VNC server
            if (vnc_proxies[i].server && !vnc_proxies[i].server->is_closing) {
                mg_send(vnc_proxies[i].server, wm->data.buf, wm->data.len);
            }
            return;
        }
    }
}

static void execute_shell_script(struct mg_connection *c, const char *script_name) {
  char script_path[1024];
  char command[2048];
  char output[8192] = {0};
  FILE *script_output;
  size_t bytes_read = 0;
  char buffer[1024];
  
  // Validate script name to prevent command injection
  if (!script_name || strlen(script_name) == 0 || 
      strchr(script_name, '/') != NULL || 
      strchr(script_name, '\\') != NULL || 
      strchr(script_name, '"') != NULL || 
      strchr(script_name, '\'') != NULL || 
      strchr(script_name, ';') != NULL || 
      strchr(script_name, '|') != NULL || 
      strchr(script_name, '&') != NULL) {
    mg_http_reply(c, 400, "Content-Type: application/json\r\n", 
                 "{\"status\":\"error\",\"message\":\"Invalid script name\"}\n");
    return;
  }
  
  // Check if script exists and has .sh extension
  snprintf(script_path, sizeof(script_path), "%s/scripts/%s", s_web_root, script_name);
  
  if (access(script_path, F_OK) != 0 || !strstr(script_name, ".sh")) {
    mg_http_reply(c, 404, "Content-Type: application/json\r\n", 
                 "{\"status\":\"error\",\"message\":\"Script not found\"}\n");
    return;
  }
  
  // Execute the script with nohup to allow it to continue running after the request completes
  //snprintf(command, sizeof(command), "nohup sudo /bin/bash %s > /dev/null 2>&1 &", script_path);
  // Run as pi and not root
  snprintf(command, sizeof(command), "nohup /bin/bash %s > /dev/null 2>&1 &", script_path); 

  if (webserver_debug_enabled) {
    printf("Executing script: %s\n", command);
  }
  
  // Execute the command
  int result = system(command);
  
  if (result == 0) {
    mg_http_reply(c, 200, "Content-Type: application/json\r\n", 
                 "{\"status\":\"success\",\"message\":\"Executing script: %s\"}\n", 
                 script_name);
  } else {
    mg_http_reply(c, 500, "Content-Type: application/json\r\n", 
                 "{\"status\":\"error\",\"message\":\"Failed to execute script\"}\n");
  }
}

static void web_despatcher(struct mg_connection *c, struct mg_ws_message *wm){
	// Check if this is a VNC proxy WebSocket
    for (int i = 0; i < MAX_VNC_PROXIES; i++) {
        if (vnc_proxies[i].active && vnc_proxies[i].client == c) {
            // Forward data from WebSocket client to VNC server
            if (vnc_proxies[i].server && !vnc_proxies[i].server->is_closing) {
                mg_send(vnc_proxies[i].server, wm->data.buf, wm->data.len);
            }
            return;
        }
    }
    
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
	else if (!strcmp(field, "BFO")) {
		char buff[1200];
		sprintf(buff, "bfo %s", value);
		remote_execute(buff);
		get_updates(c, 0);
	}
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
    } else if (strncmp(hm->uri.buf, "/cgi-bin/", 9) == 0) {
      // Check if this is a PHP file
      char uri[256];
      mg_url_decode(hm->uri.buf, hm->uri.len, uri, sizeof(uri), 0);
      
      // Check if the URI ends with .php
      size_t uri_len = strlen(uri);
      if (uri_len > 4 && strcmp(uri + uri_len - 4, ".php") == 0) {
        // Handle PHP files in cgi-bin directory
        char file_path[1024];
        snprintf(file_path, sizeof(file_path), "%s%s", s_web_root, uri);
      
        if (webserver_debug_enabled) {
          printf("PHP request: %s\n", file_path);
        }
        
        // Execute PHP directly
        char command[1500];
        snprintf(command, sizeof(command),
                "cd %s && php -f %s",
                s_web_root,
                file_path);
        
        if (webserver_debug_enabled) {
          printf("PHP command: %s\n", file_path);
          printf("Running command: %s\n", command);
        }
        
        // Execute the command and capture output
        FILE *php_output = popen(command, "r");
        if (php_output) {
          char output[16384] = "";
          size_t bytes_read = 0;
          size_t chunk_size;
          char buffer[1024];
          
          // Read all output
          while ((chunk_size = fread(buffer, 1, sizeof(buffer) - 1, php_output)) > 0) {
            if (bytes_read + chunk_size < sizeof(output) - 1) {
              memcpy(output + bytes_read, buffer, chunk_size);
              bytes_read += chunk_size;
            } else {
              break; // Prevent buffer overflow
            }
          }
          output[bytes_read] = '\0';
          pclose(php_output);
          
          if (webserver_debug_enabled) {
            printf("PHP output (%zu bytes): %s\n", bytes_read, output);
          }
          
          // Send the response
          mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", output);
        } else {
          if (webserver_debug_enabled) {
            printf("Failed to execute PHP script: %s\n", file_path);
          }
          mg_http_reply(c, 500, "", "Failed to execute PHP script\n");
        }
      }
    } else {
      // Check if this is a script execution request or HTML file request
      struct mg_http_message *hm = (struct mg_http_message *) ev_data;
      if (mg_match(hm->uri, mg_str("/vnc-proxy"), NULL)) {
        // Handle VNC proxy request
        char vnc_port_str[16] = {0};
        int vnc_port = 5901;  // Default VNC port
        
        // Get VNC port from query parameter
        mg_http_get_var(&hm->query, "vnc_port", vnc_port_str, sizeof(vnc_port_str));
        if (vnc_port_str[0] != '\0') {
            vnc_port = atoi(vnc_port_str);
        }
        
        if (vnc_port <= 0 || vnc_port > 65535) {
            // Invalid port number
            mg_http_reply(c, 400, "Content-Type: application/json\r\n", 
                         "{\"status\":\"error\",\"message\":\"Invalid VNC port\"}\n");
        } else {
            // Create a new VNC proxy
            create_vnc_proxy(c, vnc_port);
        }
      } else if (mg_match(hm->uri, mg_str("/vnc-ws"), NULL)) {
        // Upgrade to WebSocket for VNC proxy
        mg_ws_upgrade(c, hm, NULL);
      } else if (mg_match(hm->uri, mg_str("/execute-script"), NULL)) {
        // Handle script execution request
        char script_name[256] = {0};
        struct mg_str *script_param = mg_http_get_header(hm, "X-Script-Name");
        
        // First try to get script name from header
        if (script_param != NULL && script_param->len > 0) {
          // Copy the header value to our buffer
          int len = script_param->len < sizeof(script_name) - 1 ? script_param->len : sizeof(script_name) - 1;
          // Use memcpy with the raw data from the mg_str
          memcpy(script_name, script_param->buf, len);
          script_name[len] = '\0';
        } else {
          // Try to get script name from query parameter
          mg_http_get_var(&hm->query, "script", script_name, sizeof(script_name));
        }
        
        // If still no script name, try to get it from form data (POST)
        if (script_name[0] == '\0' && hm->body.len > 0) {
          mg_http_get_var(&hm->body, "script", script_name, sizeof(script_name));
        }
        
        if (script_name[0] != '\0') {
          execute_shell_script(c, script_name);
        } else {
          mg_http_reply(c, 400, "Content-Type: application/json\r\n", 
                       "{\"status\":\"error\",\"message\":\"No script specified\"}\n");
        }
      } else if (mg_match(hm->uri, mg_str("/app-list"), NULL)) {
      // Handle app list request - returns detailed information about available applications
      char output[8192] = "["; // Larger buffer for app details
      char cmd[512];
      FILE *fp;
      int first_app = 1;
      
      // Directory containing start scripts
      char scripts_dir[256];
      snprintf(scripts_dir, sizeof(scripts_dir), "%s/scripts", s_web_root);
      
      // Command to list all start_*.sh files
      snprintf(cmd, sizeof(cmd), "find %s -name 'start_*.sh' -type f -printf '%%f\n'", scripts_dir);
      
      fp = popen(cmd, "r");
      if (fp != NULL) {
        char script_name[128];
        
        // Process each start script
        while (fgets(script_name, sizeof(script_name), fp) != NULL) {
          // Remove newline character
          script_name[strcspn(script_name, "\n")] = 0;
          
          // Extract app name from script name (remove 'start_' prefix and '.sh' suffix)
          char app_name[128] = "";
          if (strncmp(script_name, "start_", 6) == 0) {
            strncpy(app_name, script_name + 6, sizeof(app_name) - 1);
            app_name[strcspn(app_name, ".")] = 0; // Remove .sh extension
            
            // Skip novnc_proxy as it's a helper script, not an application
            if (strcmp(app_name, "novnc_proxy") == 0) {
              continue;
            }
            
            // Determine VNC port and WebSocket port for this app
            int vnc_port = 5900;  // Default port
            int ws_port = 6080;   // Default WebSocket port
            
            // Try to extract port and display information from the start script
            char script_path[512];
            snprintf(script_path, sizeof(script_path), "%s/scripts/start_%s.sh", s_web_root, app_name);
            
            FILE *script_file = fopen(script_path, "r");
            if (script_file != NULL) {
              char line[512];
              while (fgets(line, sizeof(line), script_file) != NULL) {
                // Look for VNC_PORT=xxxx in the script
                if (strstr(line, "VNC_PORT=") != NULL) {
                  char *port_str = strstr(line, "VNC_PORT=") + 9; // Skip "VNC_PORT="
                  vnc_port = atoi(port_str);
                }
                
                // Look for WS_PORT=xxxx in the script
                if (strstr(line, "WS_PORT=") != NULL) {
                  char *port_str = strstr(line, "WS_PORT=") + 8; // Skip "WS_PORT="
                  ws_port = atoi(port_str);
                }
                
                // Look for DISPLAY_NUM=xxxx in the script (for future use)
                if (strstr(line, "DISPLAY_NUM=") != NULL) {
                  char *display_str = strstr(line, "DISPLAY_NUM=") + 12; // Skip "DISPLAY_NUM="
                  // We could store this for future use if needed
                  // int display_num = atoi(display_str);
                }
              }
              fclose(script_file);
            }
            
            // Fallback to known applications if ports weren't found in the script
            // if (vnc_port == 5900 && ws_port == 6080) {
            //   if (strcmp(app_name, "wsjtx") == 0) {
            //     vnc_port = 5901;
            //     ws_port = 6081;
            //   } else if (strcmp(app_name, "wsjtx_jb") == 0) {
            //     vnc_port = 5901;
            //     ws_port = 6081;
            //   } else if (strcmp(app_name, "fldigi") == 0) {
            //     vnc_port = 5902;
            //     ws_port = 6082;
            //   } else if (strcmp(app_name, "js8call") == 0) {
            //     vnc_port = 5903;
            //     ws_port = 6083;
            //   } else if (strcmp(app_name, "main_vnc") == 0) {
            //     vnc_port = 5900;
            //     ws_port = 6080;
            //   } else {
            //     // For new applications, use a dynamic port assignment
            //     // This is a simple algorithm that assigns ports based on the app name
            //     // to ensure consistency between restarts
            //     unsigned int hash = 0;
            //     for (int i = 0; app_name[i] != '\0'; i++) {
            //       hash = hash * 31 + app_name[i];
            //     }
                
            //     // Use the hash to generate a port number in the range 5904-5999
            //     // This avoids conflicts with the known applications
            //     vnc_port = 5904 + (hash % 95); // 95 = 5999 - 5904
            //     ws_port = vnc_port + 180;      // Follow the pattern of adding 180
            //   }
            // }
            
            // Look for WIDGET_LABEL in the script file
            char widget_label[128] = "";
            int found_widget_label = 0;
            
            FILE *label_file = fopen(script_path, "r");
            if (label_file != NULL) {
              char line[512];
              while (fgets(line, sizeof(line), label_file) != NULL) {
                // Look for WIDGET_LABEL="xxxx" in the script
                if (strstr(line, "WIDGET_LABEL=") != NULL) {
                  char *label_str = strstr(line, "WIDGET_LABEL=") + 13; // Skip "WIDGET_LABEL="
                  
                  // Extract the quoted string
                  char *start = strchr(label_str, '"');
                  if (start != NULL) {
                    start++; // Skip the opening quote
                    char *end = strchr(start, '"');
                    if (end != NULL) {
                      int len = end - start;
                      if (len < sizeof(widget_label)) {
                        strncpy(widget_label, start, len);
                        widget_label[len] = '\0';
                        found_widget_label = 1;
                      }
                    }
                  }
                }
              }
              fclose(label_file);
            }
            
            // If no WIDGET_LABEL found, format the app name for display (capitalize first letter, replace underscores with spaces)
            char display_name[128] = "";
            if (found_widget_label) {
              strcpy(display_name, widget_label);
            } else {
              strcpy(display_name, app_name);
              
              // Capitalize first letter
              if (display_name[0] != '\0') {
                display_name[0] = toupper(display_name[0]);
              }
              
              // Replace underscores with spaces
              for (int i = 0; display_name[i] != '\0'; i++) {
                  if (display_name[i] == '_') {
                      display_name[i] = ' ';
                  }
              }
            }
            
            // Add comma if not the first app
            if (!first_app) {
              strcat(output, ",");
            } else {
              first_app = 0;
            }
            
            // Add app details to JSON
            char app_details[512];
            snprintf(app_details, sizeof(app_details), 
                     "{\"id\":\"%s\",\"name\":\"%s\",\"vncPort\":%d,\"wsPort\":%d}", 
                     app_name, display_name, vnc_port, ws_port);
            strcat(output, app_details);
          }
        }
        pclose(fp);
      }
      
      // Close JSON array
      strcat(output, "]");
      
      // Send the response
      mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", output);
      } else if (mg_match(hm->uri, mg_str("/app-status"), NULL)) {
      // Handle app status request
      char output[4096] = "{"; // Increased buffer size for more applications
      char cmd[512];
      FILE *fp;
      int first_app = 1;
      
      // Directory containing start scripts
      char scripts_dir[256];
      snprintf(scripts_dir, sizeof(scripts_dir), "%s/scripts", s_web_root);
      
      // Command to list all start_*.sh files
      snprintf(cmd, sizeof(cmd), "find %s -name 'start_*.sh' -type f -printf '%%f\n'", scripts_dir);
      
      fp = popen(cmd, "r");
      if (fp != NULL) {
        char script_name[128];
        
        // Process each start script
        while (fgets(script_name, sizeof(script_name), fp) != NULL) {
          // Remove newline character
          script_name[strcspn(script_name, "\n")] = 0;
          
          // Extract app name from script name (remove 'start_' prefix and '.sh' suffix)
          char app_name[128] = "";
          if (strncmp(script_name, "start_", 6) == 0) {
            strncpy(app_name, script_name + 6, sizeof(app_name) - 1);
            app_name[strcspn(app_name, ".")] = 0; // Remove .sh extension
            
            // Skip novnc_proxy as it's a helper script, not an application
            if (strcmp(app_name, "novnc_proxy") == 0) {
              continue;
            }
            
            int app_running = 0;
            
            // Special case for main_vnc which needs a different check
            if (strcmp(app_name, "main_vnc") == 0) {
              // Check Main VNC using ps and grep
              FILE *app_fp = popen("ps aux | grep 'x11vnc.*-rfbport 5900' | grep -v grep > /dev/null && echo 1 || echo 0", "r");
              if (app_fp != NULL) {
                char result[10];
                if (fgets(result, sizeof(result), app_fp) != NULL) {
                  app_running = (result[0] == '1');
                }
                pclose(app_fp);
              }
              
              // Double check with the PID file
              app_fp = popen("[ -f /tmp/main_x11vnc.pid ] && kill -0 $(cat /tmp/main_x11vnc.pid) 2>/dev/null && echo 1 || echo 0", "r");
              if (app_fp != NULL) {
                char result[10];
                if (fgets(result, sizeof(result), app_fp) != NULL) {
                  // Only set to true if both checks pass, or keep existing value if already false
                  if (app_running) {
                    app_running = (result[0] == '1');
                  }
                }
                pclose(app_fp);
              }
            } else {
              // For other applications, use pgrep to check if they're running
              // Extract the actual process name from app_name (e.g., wsjtx, fldigi, js8call)
              char process_name[128];
              strcpy(process_name, app_name);
              
              // Check if the application is running
              char check_cmd[256];
              FILE *app_fp;
              char result[10];
              app_running = 0;  // Start with not running
              
              // Check if the PID file exists and process is running
              snprintf(check_cmd, sizeof(check_cmd), "[ -f /tmp/%s_app.pid ] && ps -p $(cat /tmp/%s_app.pid) > /dev/null 2>&1 && echo 1 || echo 0", app_name, app_name);
              app_fp = popen(check_cmd, "r");
              if (app_fp != NULL) {
                  if (fgets(result, sizeof(result), app_fp) != NULL) {
                      app_running = (result[0] == '1');
                  }
                  pclose(app_fp);
              }
            }
            
            // Add comma if not the first app
            if (!first_app) {
              strcat(output, ",");
            } else {
              first_app = 0;
            }
            
            // Add app status to JSON
            char app_status[256];
            snprintf(app_status, sizeof(app_status), "\"%s\":%s", app_name, app_running ? "true" : "false");
            strcat(output, app_status);
          }
        }
        pclose(fp);
      }
      
      // Close JSON object
      strcat(output, "}");
      
      // Send the response
      mg_http_reply(c, 200, "Content-Type: application/json\r\n", "%s", output);
      } else if (mg_match(hm->uri, mg_str("/index.html"), NULL) || 
                 mg_match(hm->uri, mg_str("/"), NULL)) {
        // This is a request for the main index.html file
        char file_path[1024];
        snprintf(file_path, sizeof(file_path), "%s/index.html", s_web_root);
        
        // Read the file content
        size_t content_size;
        char *content = read_file_content(file_path, &content_size);
        
        if (content) {
          // Process the content to replace version string
          size_t new_size;
          char *processed_content = process_dynamic_content(content, content_size, &new_size);
          
          if (processed_content) {
            // Send the processed content
            mg_http_reply(c, 200, "Content-Type: text/html\r\n", "%.*s", (int)new_size, processed_content);
            free(processed_content);
          } else {
            // If processing failed, serve the original content
            mg_http_reply(c, 200, "Content-Type: text/html\r\n", "%.*s", (int)content_size, content);
          }
          
          free(content);
        } else {
          // If file reading failed, serve 404
          mg_http_reply(c, 404, "", "Not found\n");
        }
      } else {
        // Serve other static files normally
        struct mg_http_serve_opts opts = {.root_dir = s_web_root};
        mg_http_serve_dir(c, ev_data, &opts);
      }
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
    } else if (wm->flags == WEBSOCKET_OP_BINARY) {
      // Check if this is a VNC proxy WebSocket
      int is_vnc_ws = 0;
      for (int i = 0; i < MAX_VNC_PROXIES; i++) {
        if (vnc_proxies[i].active && vnc_proxies[i].client == c) {
          is_vnc_ws = 1;
          handle_vnc_ws(c, wm);
          break;
        }
      }
      
      // If not a VNC proxy WebSocket, check if it's a browser microphone
      if (!is_vnc_ws) {
        // Process browser microphone data
        int16_t *audio_samples = (int16_t *)wm->data.buf;
        int sample_count = wm->data.len / sizeof(int16_t);
        
        // Pass the browser microphone data to the audio processing chain
        browser_mic_input(audio_samples, sample_count);
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

// Function to check if the only active connection is from localhost (127.0.0.1)
int is_localhost_connection_only() {
  // If no connections are active, return false
  if (active_websocket_connections == 0) {
    return 0;
  }
  
  // Check if all active connections are from localhost
  for (int i = 0; i < MAX_WS_CONNECTIONS; i++) {
    if (ws_connections[i].active) {
      // If any connection is not from localhost, return false
      if (strcmp(ws_connections[i].ip_addr, "127.0.0.1") != 0) {
        return 0;
      }
    }
  }
  
  // If we get here, all active connections are from localhost
  return 1;
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
	pthread_create( &webserver_thread, NULL, webserver_thread_function,
		(void*)NULL);
}
