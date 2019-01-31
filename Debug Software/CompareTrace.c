#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

int main (int argc, char* argv[]) {
	
	if (argc != 3) {
		printf("Usage: %s <Pulpino_TraceFile> <Klessydra_TraceFile>\n", argv[0]);
		exit(EXIT_FAILURE); }
	FILE *PULP;	
	PULP = fopen(argv[1], "r");
	if (!PULP) {
		printf("Couldn't open Pulpino file %s\n", argv[1]);
		exit(EXIT_FAILURE); }
	FILE *KLESS;
	KLESS = fopen(argv[2], "r");
	if (!KLESS) {
		printf("Couldn't open Klessydra file %s\n", argv[2]);
		exit(EXIT_FAILURE); }
	
	char *line = malloc(128 * sizeof(char));
	char PULPpc[8], KLESSpc[8];
	char bin[20];
	size_t len;
	int row = 2;
	int i; 
	
	//Erase first row from Pulpino TraceFile
	getline(&line, &len, PULP);		
	while (getline(&line, &len, PULP) != -1) {
		strcpy(bin, strtok(line, "             "));
		strcpy(bin, strtok(NULL, " "));	
		strcpy(PULPpc, strtok(NULL, " "));
		
		//Check if everything is Uppercase
		for (i=0; i<8; i++)
			PULPpc[i] = toupper(PULPpc[i]);

		//Read Klessydra Program Counter of Instructions
		fscanf(KLESS, "%s", KLESSpc);	
		if( strncmp(PULPpc, KLESSpc, 8) == 0) 
			row++;
		else
			break;	//if IPC are not equal, exit from loop
		fscanf(KLESS, "%s", bin);
		fscanf(KLESS, "%s", bin);
		fscanf(KLESS, "%s", bin);
		}
	
	printf("PULP row: %d\n", row);				//Write first row where TraceFiles begin to differ
	printf("KLES row: %d\n", (row-1)*4-3);		//Adjust index for Klessydra TraceFile
	
	fclose(PULP);
	fclose(KLESS);
	return EXIT_SUCCESS;
}
