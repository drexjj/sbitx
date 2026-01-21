#ifndef ADIF_BROADCAST_H
#define ADIF_BROADCAST_H

// Initialize ADIF UDP broadcast (reads settings from fields)
int adif_broadcast_init(void);

// Broadcast the most recently logged QSO
int adif_broadcast_qso(void);

// Cleanup broadcast socket (called on exit)
void adif_broadcast_close(void);

#endif // ADIF_BROADCAST_H
