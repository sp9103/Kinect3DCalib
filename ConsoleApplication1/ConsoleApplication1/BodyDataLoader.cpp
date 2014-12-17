#include "BodyDataLoader.h"


BodyDataLoader::BodyDataLoader(void)
{
	Datafp = NULL;
}


BodyDataLoader::~BodyDataLoader(void)
{
}

void BodyDataLoader::OpenDataFile(char *filename, char mod){
	if(mod == 'w'){
		Datafp = fopen(filename, "w");
	}
	else if(mod == 'r'){
		Datafp = fopen(filename, "w");
	}else{
		printf("Invalid File Open mode!\n");
		printf("-w : write mode\n");
		printf("-r : Read mode\n");
		
		return;
	}

	if(Datafp == NULL){
		printf("Can not open file\n");
	}
}

void BodyDataLoader::CloseDataFile(){
	if(Datafp != NULL){
		fclose(Datafp);
		Datafp = NULL;
	}
}