int endStopPins[] = {31,33,35,37,39,41,43,45,47,49,51}; //Metti tutti pin degli endstops
bool endStopHome[M_COUNT];

void initEndStops(){
	for(int i=0; i < M_COUNT; i++ ){
		pinMode(endStopPins[i],INPUT_PULLUP);
	}
}

void gotoHome(){
	int counter = 0;

	for(int i=0; i < M_COUNT; i++ ){
		endStopHome[i] = false;
	}

	int i=-1;
	while(counter < M_COUNT){

		i++;
		i = i % M_COUNT;

		if( !endStopHome[i] && !digitalRead(endStopPins[i]) ){ //digitalRead funziona se lo switch quando é premuto é in LOW perché su GND altrimenti togli !
			endStopHome[i] = true;
			counter++;
			continue;
		}

		if(endStopHome[i]) continue;

		motor[i]->onestep(FORWARD,SINGLE);

		// delay(1); //non so se serve un delay proba prima senza

	}

}