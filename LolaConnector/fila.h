#ifndef FILA_H
#define FILA_H

#include <vector>

class Fila {
private:
    std::vector<float> data;

public:
    Fila();
    bool isEmpty();
    void enf(int item);
    int desenf();
    int front();
    int size();
    void freshFila(float buffer);
    float filter();
};

#endif  // QUEUE_H
