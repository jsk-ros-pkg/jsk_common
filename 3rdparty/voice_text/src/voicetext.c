/***********************************************
 * Tts sample program
 * made by voiceware, 2006.02.24
 * modified by k-okada 2012.04.04
 ***********************************************/
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "/usr/vt/sayaka/M16/inc/vt_jpn.h"

int main(int argc, char *argv[])
{
  int infoval;
  FILE *fp;
  char input_string[65536];
  char  input_fname[1024]="";
  char output_fname[1024]="/tmp/toFile.wav";

  int ch;
  extern char *optarg;
  extern int optind, opterr;

  while ((ch = getopt(argc, argv, "o:")) != -1) {
    switch (ch) {
    case 'o':
      strncpy(output_fname,optarg,1024);
      break;
    default:
      fprintf(stderr, "Usage : %s [-o output_file] [intput_file]\n", argv[0]);
      exit(1);
    }
  }
  if (optind < argc ) {
    strncpy(input_fname,argv[optind],1024);
  }
  fprintf(stderr, "[INFO  %s] Input file  : %s\n", argv[0], input_fname);
  fprintf(stderr, "[INFO  %s] Output file : %s\n", argv[0], output_fname);

  if (( fp = fopen(input_fname, "r") ) == NULL ) {
    fprintf(stderr, "[ERROR %s] File open error %s\n", argv[0], input_fname); 
    exit(1);
  }
  while (fgets(input_string+strlen(input_string), 65536-strlen(input_string),fp)!=NULL) ;


  /****************************************/
  /* Tts Initialize                       */
  /* Call Vtprintf("%s",buf);
_Loadtts_Jpn ()               */
  /****************************************/
  if (VT_GetTTSInfo_JPN(VT_LOAD_SUCCESS_CODE, NULL, &infoval, sizeof(int)) != VT_INFO_SUCCESS) exit(1);
  if (VT_LOADTTS_JPN((int)NULL, -1, "/usr/vt/sayaka/M16/", NULL) != infoval) exit(1);


  /****************************************/
  /* TTS File API                              */
  /* call VT_TextToFile_JPN ()               */
  /****************************************/
  if (VT_TextToFile_JPN (VT_FILE_API_FMT_S16PCM_WAVE , input_string, output_fname, -1, -1, -1, -1, -1, -1, -1) != VT_FILE_API_SUCCESS)
    VT_UNLOADTTS_JPN (-1), exit (1);

  /****************************************/
  /* TTS Exit                                   */
  /* call VT_UNLOADTTS_JPN ()                */
  /****************************************/
  VT_UNLOADTTS_JPN (-1);
  exit (0);
}
