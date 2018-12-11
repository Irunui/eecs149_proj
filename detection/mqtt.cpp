/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

#include "mqtt.h"

string publish_kobuki(Kobuki k) {
    return  std::to_string(k.id)+";"+std::to_string(k.x)+";"+std::to_string(k.y)+";"+std::to_string(k.angle)+";"+std::to_string(k.timestamp);
}