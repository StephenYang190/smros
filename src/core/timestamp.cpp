//
// Created by tongda on 2022/1/7.
//

#include "timestamp.h"

Timestamp::Timestamp() {
    timestamp_ = 0;
}

int Timestamp::getCurrentime() {
    return timestamp_;
}

bool Timestamp::nextTime() {
    timestamp_++;
    return false;
}
