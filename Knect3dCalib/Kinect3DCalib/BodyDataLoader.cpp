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
		Datafp = fopen(filename, "wb");
	}
	else if(mod == 'r'){
		Datafp = fopen(filename, "rb");
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

void BodyDataLoader::ReadData(BodyJoint *dst){
	fread(dst, sizeof(BodyJoint), 1, Datafp);
}

void BodyDataLoader::WriteData(BodyJoint *src){
	fwrite(src,  sizeof(BodyJoint), 1, Datafp);
}

void BodyDataLoader::ReadAllData(){
	BodyJoint temp;
	int i = 0;

	while(!feof(Datafp)){
		ReadData(&temp);
		BodyDatalist.push_back(temp);

		/*if(feof(Datafp))
			break;*/
	}
	printf("Data Load Complete!\n");
}

void BodyDataLoader::GetBodyData(BodyJoint *dst){
	if(BodyDatalist.empty()){
		printf("error - Data Vector is empty.\n");
		return;
	}

	std::list<BodyJoint>::iterator it;
	it = BodyDatalist.begin();
	*dst  = *it;
	BodyDatalist.pop_front();
	/**dst = *BodyDatalist.begin();
	BodyDatalist.pop_front();*/
}

int BodyDataLoader::GetDataCount(){
	return BodyDatalist.size();
}