#include "utils.h"
#include <unistd.h>
#include <string.h>
#include <fstream>
#include <sstream>

void printArrayI(const char *name, const int32_t *data, uint32_t size)
{
  printf("%s: [", name);
  for (uint32_t i = 0; i < size - 1; i++)
  {
    printf("%d, ", data[i]);
  }
  printf("%d]\n", data[size - 1]);
}

void printArrayF(const char *name, const double *data, uint32_t size)
{
  printf("%s: [", name);
  for (uint32_t i = 0; i < size - 1; i++)
  {
    printf("%f, ", data[i]);
  }
  printf("%f]\n", data[size - 1]);
}

int32_t thread_rt()
{
  int32_t ret;
  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
  ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
  if (ret != 0)
  {
    printf("Failed to set rt of thread %d. %s\n", int(pthread_self()), strerror(ret));
  }
  return ret;
}

int32_t process_rt()
{
  int32_t ret;
  pid_t pid = getpid();
  struct sched_param param;
  param.sched_priority = sched_get_priority_max(SCHED_FIFO); // SCHED_RR
  ret = sched_setscheduler(pid, SCHED_FIFO, &param);
  if (ret != 0)
  {
    printf("Failed to set rt of process %d. %s\n", pid, strerror(ret));
  }
  return ret;
}

bool readCsvData(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data)
{
  std::ifstream inFile(file_name);
  if (!inFile.is_open())
  {
    std::cout << "Error:" << __FILE__ << " in line " << __LINE__ << ", Failed to open file!\n";
    return false;
  }

  std::string line;
  if (skip_header)
  {
    std::getline(inFile, line);
  }

  while (std::getline(inFile, line))
  {
    std::vector<double> line_vec;
    std::string number;
    std::istringstream readstr(line);
    while (std::getline(readstr, number, ','))
    {
      line_vec.push_back(atof(number.c_str()));
    }
    data.push_back(line_vec);
  }
  return true;
}

int kbhit(void)
{

  fd_set rfds;
  struct timeval tv;
  int retval;

  /* Watch stdin (fd 0) to see when it has input. */
  FD_ZERO(&rfds);
  FD_SET(0, &rfds);
  /* Wait up to five seconds. */
  tv.tv_sec = 0;
  tv.tv_usec = 0;

  retval = select(1, &rfds, NULL, NULL, &tv);
  /* Don't rely on the value of tv now! */

  if (retval == -1)
  {
    perror("select()");
    return 0;
  }
  else if (retval)
    return 1;
  /* FD_ISSET(0, &rfds) will be true. */
  else
    return 0;
  return 0;
}
