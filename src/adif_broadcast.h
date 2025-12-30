#ifndef ADIF_BROADCAST_H
#define ADIF_BROADCAST_H

// Initialize ADIF UDP broadcast (called on first use)
int adif_broadcast_init(const char *ip, int port);

// Broadcast the most recently logged QSO
int adif_broadcast_qso(void);

// Cleanup broadcast socket (called on exit)
void adif_broadcast_close(void);

#endif // ADIF_BROADCAST_H
