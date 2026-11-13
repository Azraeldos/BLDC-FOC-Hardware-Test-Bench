#pragma once

#ifdef __cplusplus
extern "C" {
#endif

void user_setup(void);   // one-shot init after MX_*_Init()
void user_loop(void);    // called every main-loop iteration

#ifdef __cplusplus
}
#endif