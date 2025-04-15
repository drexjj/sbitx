#include <stdio.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#include <stdlib.h>
#include <errno.h>
#include <fcntl.h>
#include <complex.h>
#include <math.h>
#include <fcntl.h>
#include <complex.h>
#include <fftw3.h>
#include <pthread.h>
#include "sdr.h"
#include "sdr_ui.h"

#define DEBUG 0
#define MAX_CLIENTS 10
static int client_sockets[MAX_CLIENTS] = {0};
static int welcome_socket = -1;
volatile int running = 1; // Control flag for the listener thread

pthread_mutex_t client_mutex = PTHREAD_MUTEX_INITIALIZER;

void handle_client_disconnection(int i, int sd) {
    pthread_mutex_lock(&client_mutex);
    if (client_sockets[i] == sd) {
        printf("Closing client socket %d at index %d\n", sd, i);
        close(sd);
        client_sockets[i] = 0; // Mark as available
    }
    pthread_mutex_unlock(&client_mutex);
}

//copied from gqrx on github
static char dump_state_response[] =
        /* rigctl protocol version */
        "0\n"
        /* rigctl model */
        "2\n"
        /* ITU region */
        "2\n"
        /* RX/TX frequency ranges
         * start, end, modes, low_power, high_power, vfo, ant
         *  start/end - Start/End frequency [Hz]
         *  modes - Bit field of RIG_MODE's (AM|AMS|CW|CWR|USB|LSB|FM|WFM)
         *  low_power/high_power - Lower/Higher RF power in mW,
         *                         -1 for no power (ie. rx list)
         *  vfo - VFO list equipped with this range (RIG_VFO_A)
         *  ant - Antenna list equipped with this range, 0 means all
         *  FIXME: limits can be gets from receiver::get_rf_range()
         */
        "100000 30000000 0x2ef -1 -1 0x1 0x0\n"
        /* End of RX frequency ranges. */
        "0 0 0 0 0 0 0\n"
        /* End of TX frequency ranges. The Gqrx is receiver only. */
        "0 0 0 0 0 0 0\n"
        /* Tuning steps: modes, tuning_step */
        "0xef 1\n"
        "0xef 0\n"
        /* End of tuning steps */
        "0 0\n"
        /* Filter sizes: modes, width
         * FIXME: filter can be gets from filter_preset_table
         */
        "0x82 500\n"    /* CW | CWR normal */
        "0x82 200\n"    /* CW | CWR narrow */
        "0x82 2000\n"   /* CW | CWR wide */
        "0x221 5000\n"  /* AM | AMS | FM narrow */
        "0x0c 2700\n"   /* SSB normal */
        /* End of filter sizes  */
        "0 0\n"
        /* max_rit  */
        "0\n"
        /* max_xit */
        "0\n"
        /* max_ifshift */
        "0\n"
        /* Announces (bit field list) */
        "0\n" /* RIG_ANN_NONE */
        /* Preamp list in dB, 0 terminated */
        "0\n"
        /* Attenuator list in dB, 0 terminated */
        "0\n"
        /* Bit field list of get functions */
        "0\n" /* RIG_FUNC_NONE */
        /* Bit field list of set functions */
        "0\n" /* RIG_FUNC_NONE */
        /* Bit field list of get level */
        "0x40000020\n" /* RIG_LEVEL_SQL | RIG_LEVEL_STRENGTH */
        /* Bit field list of set level */
        "0x20\n"       /* RIG_LEVEL_SQL */
        /* Bit field list of get parm */
        "0\n" /* RIG_PARM_NONE */
        /* Bit field list of set parm */
        "0\n" /* RIG_PARM_NONE */;

/*
enum rig_errcode_e {
    RIG_OK = 0,     /*!< 0 No error, operation completed successfully 
RIG_EINVAL,     /*!< 1 invalid parameter 
RIG_ECONF,      /*!< 2 invalid configuration (serial,..) 
RIG_ENOMEM,     /*!< 3 memory shortage 
RIG_ENIMPL,     /*!< 4 function not implemented, but will be 
RIG_ETIMEOUT,   /*!< 5 communication timed out 
RIG_EIO,        /*!< 6 IO error, including open failed 
RIG_EINTERNAL,  /*!< 7 Internal Hamlib error, huh! 
RIG_EPROTO,     /*!< 8 Protocol error 
RIG_ERJCTED,    /*!< 9 Command rejected by the rig 
RIG_ETRUNC,     /*!< 10 Command performed, but arg truncated 
RIG_ENAVAIL,    /*!< 11 Function not available 
RIG_ENTARGET,   /*!< 12 VFO not targetable 
RIG_BUSERROR,   /*!< 13 Error talking on the bus 
RIG_BUSBUSY,    /*!< 14 Collision on the bus 
RIG_EARG,       /*!< 15 NULL RIG handle or any invalid pointer parameter in get arg 
RIG_EVFO,       /*!< 16 Invalid VFO 
RIG_EDOM,       /*!< 17 Argument out of domain of func 
RIG_EDEPRECATED,/*!< 18 Function deprecated 
RIG_ESECURITY,  /*!< 19 Security error 
RIG_EPOWER,     /*!< 20 Rig not powered on 
RIG_ELIMIT,     /*!< 21 Limit exceeded 
RIG_EEND        // MUST BE LAST ITEM IN LAST
};
*/

int check_cmd(char *cmd, char *token){
  if (strstr(cmd, token) == cmd)
    return 1;
  else
    return 0;
}

#define MAX_DATA 1000
char incoming_data[MAX_CLIENTS][MAX_DATA];
int incoming_ptr[MAX_CLIENTS] = {0};

void send_response(int client_socket, char *response) {
    int e = send(client_socket, response, strlen(response), 0);
#if DEBUG > 0
    printf("hamlib>>> response: [%s]\n", response);
#endif
    if (e < 0) {
        printf("Hamlib client disconnected (send). Closing socket %d.\n", client_socket);
        close(client_socket);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            if (client_sockets[i] == client_socket) {
                client_sockets[i] = 0;
                break;
            }
        }
    }
}

void send_freq(int client_socket){
  //Returns the active VFO's current frequency
  char response[20];
  sprintf(response, "%d\n", get_freq());
  send_response(client_socket, response);
}
void send_mode(int client_socket){
char mode[10];
    char response[20];
    get_field_value_by_label("MODE",mode);
    if (!strcmp(mode,"DIGI"))
      strcpy(mode,"PKTUSB");
    sprintf(response,"%s\n%i\n", mode,  get_passband_bw());
    send_response(client_socket, response);
    //printf("[%s]",response);
}
void set_mode(int client_socket, char* f) {
    char mode[10];
    char cmd[50];
    char passband[3];
    char* tok = strtok(f," ");
    if (tok != 0) {
        strcpy(mode,tok);
        tok = strtok(0," ");
        //printf("Received mode: [%s] \n", mode);
        if (tok != 0) {
            strcpy(passband,tok);
            //printf("Received bw: [%s] \n", passband);
        }
    } else {
        //We didn't receive what was expected
        send_response(client_socket, "RPRT -9\n");
        return;
    } 
    if (!strcmp(mode, "PKTUSB"))
        strcpy(mode, "DIGI");
    //printf("Mode? = '%s'\n", mode);
    const char* supported_hamlib_modes[6] = {
    "USB", "LSB", "CW", "CWR","DIGI","AM"
    };
    for (int i = 0; i < 6; i++) {
        if (!strcmp(mode, supported_hamlib_modes[i])) {
            char bw_str[10];
            sprintf(cmd, "mode %s", mode);
            cmd_exec(cmd);
            if (!strcmp(passband,"0")) {
                //passband=0 == use default BW for mode
                sprintf(bw_str, "%d", get_default_passband_bw());
	            field_set("BW", bw_str);
            }
            send_response(client_socket, "RPRT 0\n");
            return;
        }
    }
    //Unknown mode
    printf("Unknown mode passed: [%s]\n", mode);
    send_response(client_socket, "RPRT -9\n");
}
void send_rfpower(int client_socket) {
    char resp[3];
    float drive = (float)field_int("DRIVE") / (float)100;
    sprintf(resp, "%f\n", drive);
    send_response(client_socket, resp);
}
void get_vfo(int client_socket) {
    char currVFO[2];
    get_field_value_by_label("VFO", currVFO);
    //printf("Current VFO: %s\n", currVFO);
    if (currVFO[0] == 'A') {
        //active_vfo = 0;
        send_response(client_socket, "VFOA\n");
    } else {
        //active_vfo = 1;
        send_response(client_socket, "VFOB\n");
    }
}

void set_vfo(int client_socket, char* f) {
    field_set("VFO", f);
    //~ printf("[VFO%s\n]", f);
    char response[6];
    snprintf(response, sizeof(response), "VFO%s\n", f);
    send_response(client_socket, "RPRT 0\n");
}

void get_split(int client_socket) {
    char curr_split[4];
    get_field_value_by_label("SPLIT", curr_split);
    
    if (!strcmp(curr_split,"OFF")) {
        send_response(client_socket, "0\n");
        get_vfo(client_socket);
    }
    else {
        send_response(client_socket, "1\n");
        //In split mode, tx vfo is always B
        send_response(client_socket, "VFOB\n");
    }
    
}
void hamlib_set_freq(int client_socket, char *f){
#if DEBUG > 0
    printf("hamlib_set_freq func arg: [%s]\n",f);
#endif
  long freq;
  char cmd[50];
  if (!strncmp(f, "VFO", 3))
    freq = atoi(f+5);
  else
    freq = atoi(f);
	sprintf(cmd, "freq %d", freq);
#if DEBUG > 0
    printf("Send string to cmd_exec: [%s]\n",cmd);
#endif
	cmd_exec(cmd);
    send_response(client_socket, "RPRT 0\n");
}
	

void tx_control(int client_socket, int s){
	//printf("tx_control(%d)\n", s);
  if (s == 1){
    hamlib_tx(1);
  }
  if (s == 0){
    hamlib_tx(0);
  }
  if (s == -1) {
      char tx_status[100];
      sdr_request("stat:tx=1", tx_status);
      if (!strcmp(tx_status, "ok on"))
          send_response(client_socket, "1\n");
      else
          send_response(client_socket, "0\n");
      return;
  }
  send_response(client_socket, "RPRT 0\n");

}

void interpret_command(int client_socket, char* cmd) {
    if (cmd[0] == 'T' || check_cmd(cmd, "\\set_ptt")) {
        if (strchr(cmd, '0'))
            tx_control(client_socket, 0); //if there is a zero in it, we are to rx
        else
            tx_control(client_socket, 1); //this is a shaky way to do it, who has the time to parse?
    } 
    else if (check_cmd(cmd, "F") || check_cmd(cmd, "\\set_freq"))
        if (cmd[0] == 'F')
            hamlib_set_freq(client_socket, cmd + 2);
        else
            hamlib_set_freq(client_socket, cmd + 10);
    else if (check_cmd(cmd, "\\chk_vfo"))
        //Lets not default to VFO mode
        send_response(client_socket, "0\n");
    else if (check_cmd(cmd, "\\dump_state"))
        send_response(client_socket, dump_state_response);
    else if (check_cmd(cmd, "V VFOA") || check_cmd(cmd, "\\set_vfo VFOA")) {
        set_vfo(client_socket, "A");
    }
    else if (check_cmd(cmd, "V VFOB") || check_cmd(cmd, "\\set_vfo VFOB")) {
        set_vfo(client_socket, "B");
    } else if (check_cmd(cmd, "v")|| check_cmd(cmd, "\\get_vfo")) {
        get_vfo(client_socket);
    } else if (cmd[0] == 'm' || check_cmd(cmd, "\\get_mode"))
        send_mode(client_socket);
    else if (cmd[0] == 'M' || check_cmd(cmd, "\\set_mode")) {
        if (cmd[0] == 'M')
            set_mode(client_socket, cmd + 2);
        else
            set_mode(client_socket, cmd + 10);
    } else if (cmd[0] == 'f' || check_cmd(cmd, "\\get_freq"))
        send_freq(client_socket);
    else  if (cmd[0] == 's' || check_cmd(cmd, "\\get_split_vfo")) {
        get_split(client_socket);
        
    } else if (check_cmd(cmd, "t") || check_cmd(cmd, "\\get_ptt"))
        tx_control(client_socket, -1);
    else if (check_cmd(cmd, "\\get_powerstat"))
        send_response(client_socket, "1\n");
    else if (check_cmd(cmd, "\\get_lock_mode"))
        send_response(client_socket, "0\n");
    else if (check_cmd(cmd, "q") || check_cmd(cmd, "Q")) {
        send_response(client_socket, "RPRT 0\n");
        close(client_socket);
        client_socket = -1;
    }
    else if (check_cmd(cmd, "u TUNER"))
        send_response(client_socket, "0\n");
  else if (check_cmd(cmd, "l RFPOWER")) {
        send_rfpower(client_socket);
  }  else { 
    printf("Hamlib: Unrecognized command [%s] '%c'\n", cmd, cmd[0]);
    //Send an unimplemented response error code
    send_response(client_socket, "RPRT -11\n");
  }
}

void hamlib_handler(int client_socket, char *data, int len) {
    int client_index = -1;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] == client_socket) {
            client_index = i;
            break;
        }
    }
    if (client_index == -1) return;

    for (int i = 0; i < len; i++) {
        if (data[i] == '\n') {
            incoming_data[client_index][incoming_ptr[client_index]] = 0;
            incoming_ptr[client_index] = 0;
            interpret_command(client_socket, incoming_data[client_index]);
        } else if (incoming_ptr[client_index] < MAX_DATA) {
            incoming_data[client_index][incoming_ptr[client_index]++] = data[i];
        }
    }
}

void hamlib_start() {
    struct sockaddr_in serverAddr;
    welcome_socket = socket(PF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);

    serverAddr.sin_family = AF_INET;
    serverAddr.sin_port = htons(4532);
    serverAddr.sin_addr.s_addr = INADDR_ANY;
    memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);

    bind(welcome_socket, (struct sockaddr *)&serverAddr, sizeof(serverAddr));
    listen(welcome_socket, 5);
    printf("Server listening on port 4532\n");
    
}

int get_max_sd() {
    int max_sd = welcome_socket;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] > max_sd) {
            max_sd = client_sockets[i];
        }
    }
    return max_sd;
}

void *hamlib_slice(void *arg) {
    hamlib_start(); // Initialize the server socket here in the thread
    fd_set read_fds;
    int sd, activity, new_socket;
    struct sockaddr_in address;
    socklen_t addrlen = sizeof(address);

    while (running) {
        // Clear and set up file descriptor set for select
        FD_ZERO(&read_fds);
        FD_SET(welcome_socket, &read_fds);
        
        // Recalculate max_sd before each select call
        int max_sd = get_max_sd();

        // Add all active client sockets to the set
        pthread_mutex_lock(&client_mutex);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            sd = client_sockets[i];
            if (sd > 0) {
                FD_SET(sd, &read_fds);
            }
        }
        pthread_mutex_unlock(&client_mutex);

        struct timeval timeout;
        timeout.tv_sec = 0;
        timeout.tv_usec = 100000; // 100 ms timeout

        // Wait for activity on one of the sockets
        //printf("Waiting on select with max_sd: %d\n", max_sd);
        activity = select(max_sd + 1, &read_fds, NULL, NULL, &timeout);
        
        if (activity < 0 && errno != EINTR) {
            perror("select error");
            break; // Exit loop if select fails
        }

        // Check for new connection on welcome socket
        if (FD_ISSET(welcome_socket, &read_fds)) {
            new_socket = accept(welcome_socket, (struct sockaddr *)&address, &addrlen);
            if (new_socket >= 0) {
                printf("New client connected on socket %d\n", new_socket);
                fcntl(new_socket, F_SETFL, O_NONBLOCK);

                // Add new client socket
                pthread_mutex_lock(&client_mutex);
                int added = 0;
                for (int i = 0; i < MAX_CLIENTS; i++) {
                    if (client_sockets[i] == 0) {
                        client_sockets[i] = new_socket;
                        added = 1;
                        break;
                    }
                }
                pthread_mutex_unlock(&client_mutex);

                if (!added) {
                    printf("Too many clients connected. Rejecting new client.\n");
                    close(new_socket);
                }
            }
        }

        // Process each client socket
        pthread_mutex_lock(&client_mutex);
        for (int i = 0; i < MAX_CLIENTS; i++) {
            sd = client_sockets[i];
            if (sd > 0 && FD_ISSET(sd, &read_fds)) {
                char buffer[1024];
                int len = recv(sd, buffer, sizeof(buffer), 0);

                if (len > 0) {
                    buffer[len] = '\0';
                    //printf("Data received from client %d: %s\n", sd, buffer);
                    hamlib_handler(sd, buffer, len);
                } else if (len == 0 || (len < 0 && errno != EAGAIN && errno != EWOULDBLOCK)) {
                    printf("Client on socket %d disconnected or error occurred.\n", sd);
                    close(sd);
                    client_sockets[i] = 0; // Mark as available
                    FD_CLR(sd, &read_fds); // Remove the socket from read_fds
                }
            }

            // Additional check with getsockopt to detect disconnected sockets
            if (sd > 0) {
                int error = 0;
                socklen_t len = sizeof(error);
                int retval = getsockopt(sd, SOL_SOCKET, SO_ERROR, &error, &len);
                if (retval != 0 || error != 0) {
                    printf("Detected closed connection or error on socket %d\n", sd);
                    close(sd);
                    client_sockets[i] = 0; // Mark as available
                    FD_CLR(sd, &read_fds); // Remove the socket from read_fds
                }
            }
        }
        pthread_mutex_unlock(&client_mutex);
    }

    // Clean up on exit
    close(welcome_socket);
    pthread_mutex_lock(&client_mutex);
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (client_sockets[i] > 0) {
            close(client_sockets[i]);
            client_sockets[i] = 0;
        }
    }
    pthread_mutex_unlock(&client_mutex);

    return NULL;
}



// Function to start the listener thread
void start_hamlib_listener() {
    pthread_t listener_thread;
    if (pthread_create(&listener_thread, NULL, hamlib_slice, NULL) != 0) {
        perror("Failed to create listener thread");
        exit(1);
    }
    pthread_detach(listener_thread); // Detach the thread to run independently
}

// Function to stop the listener thread
void stop_hamlib_listener() {
    running = 0; // Stop the loop in hamlib_slice
    close(welcome_socket); // Close the server socket to exit select
}

void *start_listener_thread(void *arg) {
    start_hamlib_listener(); // This starts the listener and runs it in the new thread
    return NULL;
}

// Call this function in your GTK initialization code
void initialize_hamlib() {
    pthread_t listener_thread;
    
    // Start the listener thread
    if (pthread_create(&listener_thread, NULL, start_listener_thread, NULL) != 0) {
        perror("Failed to create listener thread");
        exit(1);
    }
    
    pthread_detach(listener_thread); // Detach to ensure it doesn't block or need to be joined
}

