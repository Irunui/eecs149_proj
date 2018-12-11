/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   mqtt.h
 * Author: alex
 *
 * Created on December 3, 2018, 10:12 AM
 */

#ifndef MQTT_H
#define MQTT_H

#include <iostream>
#include <cstdlib>
#include <string>
#include <cstring>
#include <cctype>
#include <thread>
#include <chrono>
#include "mqtt/async_client.h"

using namespace std;

const string SERVER_ADDRESS	{ "tcp://localhost:1883" };
const string CLIENT_ID		{ "async_consume" };
const string TOPIC 			{ "hello" };
const string TOPIC2 			{ "kobuki" };

const auto TIMEOUT = std::chrono::seconds(1);


const int  QOS = 1;

string publish_kobuki(Kobuki k) ;

#endif /* MQTT_H */

