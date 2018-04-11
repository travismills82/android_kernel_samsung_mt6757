#ifndef _JACK_MADERA_SYSFS_CB_H
#define _JACK_MADERA_SYSFS_CB_H

#include <sound/soc.h>

extern int madera_jack_det;
extern int madera_ear_mic;

void register_madera_jack_cb(struct snd_soc_codec *codec);

#endif /*_JACK_MADERA_SYSFS_CB_H */
