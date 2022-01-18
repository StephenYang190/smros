/* Timestamp class
 * Create by Tongda Yang
 * This class is used to control whole slam time
 * */

#ifndef SRC_TIME_STAMP_H
#define SRC_TIME_STAMP_H


class Timestamp {
public:
    int timestamp_;

    Timestamp();
    ~Timestamp();
    int getCurrentime();
    bool nextTime();
};


#endif //SRC_TIME_STAMP_H
