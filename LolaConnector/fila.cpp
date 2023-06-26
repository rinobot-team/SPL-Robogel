#include <iostream>
#include "fila.h"

Fila::Fila() {}

bool Fila::isEmpty() {
    return data.empty();
}

void Fila::enf(int item) {
    data.push_back(item);
}

int Fila::desenf() {
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty");
    }
    
    int frontItem = data.front();
    data.erase(data.begin());
    return frontItem;
}

int Fila::front() {
    if (isEmpty()) {
        throw std::runtime_error("Queue is empty");
    }
    
    return data.front();
}

int Fila::size() {
    return data.size();
}

void Fila::freshFila(float buffer){
    if(data.size() < 30){
		data.push_back(buffer);
		return;
	}
	else{
		int frontItem = data.front();
        data.erase(data.begin());
		data.push_back(buffer);
		return;
	}
}

float Fila::filter(){
    int frontItem = 0;
    float dado_filter = 0;
	float n = data.size();
	for(int c = 0; c < n; c++){
        frontItem = data.front();
        data.erase(data.begin());
		dado_filter += frontItem;
	}
	return (dado_filter/n);
}
