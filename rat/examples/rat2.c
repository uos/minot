#include "../rat.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>

int main() {
  int ret = rat_init("testRat2");
  if (ret != 0) {
    printf("Error initializing rat");
    return ret;
  }

  unsigned char* var = NULL;
  unsigned char* var3 = NULL;
  var = malloc(sizeof(*var));
  var3 = malloc(sizeof(*var3));

  *var = false;
  *var3 = false;

  ret = rat_bacon_u8("var1", var, 1, 1);
  if (ret != 0) {
    printf("Error while bacon var1");
    return ret;
  }
  assert(*var == true);
  free(var);


  ret = rat_bacon_u8("var3", var3, 1, 1);
  if (ret != 0) {
    printf("Error while bacon var3");
    return ret;
  }
  assert(*var3 == false);
  free(var3);

  ret = rat_deinit();
  if (ret != 0) {
    printf("Error deinitializing rat");
    return ret;
  }


  printf("\x1b[32m\nSuccess!\x1b[0m\n");
  return 0;
}

 
