#ifndef _utils_h_
#define _utils_h_

#include <iostream>
#include <vector>

#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9))
#define LIMITING(v, min, max) ((v) > (max) ? (max) : ((v) < (min) ? (min) : (v)))

#define TO_DEGREE (180.0 / M_PI)
#define TO_RADIAN (M_PI / 180.0)

void printArrayI(const char *name, const int32_t *data, uint32_t size);
void printArrayF(const char *name, const double *data, uint32_t size);

int32_t thread_rt();
int32_t process_rt();

bool readCsvData(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data);

int kbhit(void);

#endif
