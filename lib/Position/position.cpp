#include "position.h"
#include <Arduino.h>

Position::Position(float x, float y, float z){
    this->x = x;
    this->y = y;
    this->z = z;
}

Position::Position(){
    this->x = 0.0;
    this->y = 0.0;
    this->z = 0.0;
}

float Position::getX(){
    return x;
}

float Position::getY(){
    return y;
}

float Position::getZ(){
    return z;
}

void Position::setX(float x){
    this->x = x;
}

void Position::setY(float y){
    this->y = y;
}

void Position::setZ(float z){
    this->z = z;
}

bool Position::compareValues(float t1, float t2, float df){
    if (abs(t1-t2)>df){
        return false;
    } else {
        return true;
    }
}

bool Position::isOnPosition(Position pos){
    if (compareValues(x, pos.getX(), 0.01) && compareValues(y, pos.getY(), 0.01) && compareValues(z, pos.getZ(), 0.01)){
        //Serial.println("[Position Check] (" + String(x) + ", " + String(y) + ", " + String(z) + ") ~ (" + String(pos.getX()) + ", " + String(pos.getY()) + ", " + String(pos.getZ()) + ")");
        return true;
    } else {
        //Serial.println("[Position Check] (" + String(x) + ", " + String(y) + ", " + String(z) + ") != (" + String(pos.getX()) + ", " + String(pos.getY()) + ", " + String(pos.getZ()) + ")");
        return false;
    }
}
