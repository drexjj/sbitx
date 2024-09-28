
/*
Hamlib rigcltd emulation
example rigctl connecetion command:  rigctl -m 2 -r localhost:4532

Known issues
-Can only handle one client at once currently - n1qm
*/
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
#include "sdr.h"
#include "sdr_ui.h"

#define DEBUG 0

static int welcome_socket = -1, data_socket = -1;
#define MAX_DATA 1000
char incoming_data[MAX_DATA];
int incoming_ptr;

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

void send_response(char *response){
  int e = send(data_socket, response, strlen(response), 0);
#if DEBUG > 0
        printf("hamlib>>> response: [%s]\n",response);
#endif
  //printf("[%s]", response); 
  if (e >=0) return;
  // Connection closed by client
  puts("Hamlib client disconnected (send). Restarting ...");
  close(data_socket);
  data_socket = -1;
}

void send_freq(){
  //Returns the active VFO's current frequency
  char response[20];
  sprintf(response, "%d\n", get_freq());
  send_response(response);
}
void send_mode(){
char mode[10];
    char response[20];
    get_field_value_by_label("MODE",mode);
    if (!strcmp(mode,"DIGI"))
      strcpy(mode,"PKTUSB");
    sprintf(response,"%s\n%i\n", mode,  get_passband_bw());
    send_response(response);
    //printf("[%s]",response);
}
void set_mode(char* f) {
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
        send_response("RPRT -9\n");
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
            send_response("RPRT 0\n");
            return;
        }
    }
    //Unknown mode
    printf("Unknown mode passed: [%s]\n", mode);
    send_response("RPRT -9\n");
}
void send_rfpower() {
    char resp[3];
    float drive = (float)field_int("DRIVE") / (float)100;
    sprintf(resp, "%f\n", drive);
    send_response(resp);
}
void get_vfo() {
    char currVFO[2];
    get_field_value_by_label("VFO", currVFO);
    //printf("Current VFO: %s\n", currVFO);
    if (currVFO[0] == 'A') {
        //active_vfo = 0;
        send_response("VFOA\n");
    } else {
        //active_vfo = 1;
        send_response("VFOB\n");
    }
}
void set_vfo(char* f) {
    field_set("VFO", f);
    char response[5];
    sprintf(response, "VFO%s\n", f);
    //printf("[VFO%s\n]", f);
    send_response("RPRT 0\n");


}
void get_split() {
    char curr_split[4];
    get_field_value_by_label("SPLIT", curr_split);
    
    if (!strcmp(curr_split,"OFF")) {
        send_response("0\n");
        get_vfo();
    }
    else {
        send_response("1\n");
        //In split mode, tx vfo is always B
        send_response("VFOB\n");
    }
    
}
void hamlib_set_freq(char *f){
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
    send_response("RPRT 0\n");
}
	

void tx_control(int s){
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
          send_response("1\n");
      else
          send_response("0\n");
      return;
  }
  send_response("RPRT 0\n");

}

void interpret_command(char* cmd) {
    if (cmd[0] == 'T' || check_cmd(cmd, "\\set_ptt")) {
        if (strchr(cmd, '0'))
            tx_control(0); //if there is a zero in it, we are to rx
        else
            tx_control(1); //this is a shaky way to do it, who has the time to parse?
    } 
    else if (check_cmd(cmd, "F") || check_cmd(cmd, "\\set_freq"))
        if (cmd[0] == 'F')
            hamlib_set_freq(cmd + 2);
        else
            hamlib_set_freq(cmd + 10);
    else if (check_cmd(cmd, "\\chk_vfo"))
        //Lets not default to VFO mode
        send_response("0\n");
    else if (check_cmd(cmd, "\\dump_state"))
        send_response(dump_state_response);
    else if (check_cmd(cmd, "V VFOA") || check_cmd(cmd, "\\set_vfo VFOA")) {
        set_vfo("A");
    }
    else if (check_cmd(cmd, "V VFOB") || check_cmd(cmd, "\\set_vfo VFOB")) {
        set_vfo("B");
    } else if (check_cmd(cmd, "v")|| check_cmd(cmd, "\\get_vfo")) {
        get_vfo();
    } else if (cmd[0] == 'm' || check_cmd(cmd, "\\get_mode"))
        send_mode();
    else if (cmd[0] == 'M' || check_cmd(cmd, "\\set_mode")) {
        if (cmd[0] == 'M')
            set_mode(cmd + 2);
        else
            set_mode(cmd + 10);
    } else if (cmd[0] == 'f' || check_cmd(cmd, "\\get_freq"))
        send_freq();
    else  if (cmd[0] == 's' || check_cmd(cmd, "\\get_split_vfo")) {
        get_split();
        
    } else if (check_cmd(cmd, "t") || check_cmd(cmd, "\\get_ptt"))
        tx_control(-1);
    else if (check_cmd(cmd, "\\get_powerstat"))
        send_response("1\n");
    else if (check_cmd(cmd, "\\get_lock_mode"))
        send_response("0\n");
    else if (check_cmd(cmd, "q") || check_cmd(cmd, "Q")) {
        send_response("RPRT 0\n");
        close(data_socket);
        data_socket = -1;
    }
    else if (check_cmd(cmd, "u TUNER"))
        send_response("0\n");
  else if (check_cmd(cmd, "l RFPOWER")) {
        send_rfpower();
  }  else { 
    printf("Hamlib: Unrecognized command [%s] '%c'\n", cmd, cmd[0]);
    //Send an unimplemented response error code
    send_response("RPRT -11\n");
  }
}

void hamlib_handler(char *data, int len){

  for (int i = 0; i < len; i++){
    if (data[i] == '\n'){
      incoming_data[incoming_ptr] = 0;
      incoming_ptr = 0;
#if DEBUG > 0
        printf("<<<hamlib cmd: [%s]\n",data);
#endif
      //printf("<<<hamlib cmd %s =>", data);
      interpret_command(incoming_data);
    }
    else if (incoming_ptr < MAX_DATA){
      incoming_data[incoming_ptr] = data[i]; 
      incoming_ptr++;
    }
  }
}

void hamlib_start(){
  char buffer[MAX_DATA];
  struct sockaddr_in serverAddr;
  struct sockaddr_storage serverStorage;
  socklen_t addr_size;

  welcome_socket = socket(PF_INET, SOCK_STREAM | SOCK_NONBLOCK, 0);
  
  serverAddr.sin_family = AF_INET;
  serverAddr.sin_port = htons(4532);
  //Allow connections from all interfaces
  //Shouldn't use with a public IP
  serverAddr.sin_addr.s_addr = INADDR_ANY;
  memset(serverAddr.sin_zero, '\0', sizeof serverAddr.sin_zero);  

  /*---- Bind the address struct to the socket ----*/
  bind(welcome_socket, (struct sockaddr *) &serverAddr, sizeof(serverAddr));

  /*---- Listen on the socket, with 5 max connection requests queued ----*/
  if(listen(welcome_socket,5)!=0)
    printf("hamlib listen() Error\n");
  incoming_ptr = 0;
}

void hamlib_slice(){
  struct sockaddr_storage server_storage;
  socklen_t addr_size;
  int e, len;
  char buffer[1024];

  if (data_socket == -1){
    addr_size = sizeof server_storage;
    e = accept(welcome_socket, (struct sockaddr *) &server_storage, &addr_size);
    if (e == -1)
      return;
    puts("Accepted Hamlib client\n");
    incoming_ptr = 0;
    data_socket = e;
    fcntl(data_socket, F_SETFL, fcntl(data_socket, F_GETFL) | O_NONBLOCK);
    }
        // closing blocks modified by JJ W9JES
        else {
        len = recv(data_socket, buffer, sizeof(buffer), 0);
        if (len > 0) {
            buffer[len] = '\0';
            hamlib_handler(buffer, len);
        } else if (len == 0) {
            // Connection closed by client
            puts("Hamlib client disconnected. Restarting to listen ...");
            close(data_socket);
            data_socket = -1;
        } else {
            // len < 0 indicates error
            if (errno == EAGAIN || errno == EWOULDBLOCK) {
                return;
            }
            // For other errors, close the socket
            puts("Hamlib client dropped. Restarting to listen ...");
            close(data_socket);
            data_socket = -1;     
    }
  } 
}
