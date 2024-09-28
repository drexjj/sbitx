#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

//Returns the cwd with argument 1 appended -- n1qm
char* app_cwd(const char* str1) {
    size_t len1 = strlen(str1);
	char directory[4096];

	//Get path of where this binary resides
	int readPath = readlink("/proc/self/exe", directory, 4096);
	
	//Find the last path seperator
	int lastSep = 0;
	for (int i=0;i < readPath;i++) {
		if (directory[i] == '/')
			lastSep=i;
	}

	//Trim string at last seperator if > 0
	if (lastSep > 0)
		directory[lastSep] = '\0';
	else
		directory[1]='\0';

	//return path of binary if no argument is passed
	if (len1 > 0) {
		if (str1[0] != '/')
			strcat(directory,"/");
		strcat(directory,str1);
	}
	printf("In app_cwd, arg in: %s, returning: %s\n",str1,directory);

    return strdup(directory); // Return the concatenated string
}