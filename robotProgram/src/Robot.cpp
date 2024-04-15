// Copyright (c) FRC 2053.
// Open Source Software; you can modify and/or share it under the terms of
// the MIT License file in the root of this project

#define LIBSSH_STATIC

#include <libssh/libssh.h>


int main() {
  ssh_session my_ssh_session = ssh_new();
  if (my_ssh_session == NULL)
    return -1;
  ssh_free(my_ssh_session);
  return 0;
}