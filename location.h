/*
 * location.h
 *
 *  Created on: 2017-8-19
 *      Author: wangbo
 */

#ifndef LOCATION_H_
#define LOCATION_H_

#include <stdint.h>

struct Location {
    uint8_t id;                                                 ///< command id
    uint8_t options;                                    ///< options bitmask (1<<0 = relative altitude)
    uint8_t p1;                                                 ///< param 1
    int32_t alt;                                        ///< param 2 - Altitude in centimeters (meters * 100)
    int32_t lat;                                        ///< param 3 - Lattitude * 10**7
    int32_t lng;                                        ///< param 4 - Longitude * 10**7
};



// find a rotation that is the combination of two other
// rotations. This is used to allow us to add an overall board
// rotation to an existing rotation of a sensor such as the compass
//enum Rotation           rotation_combination(enum Rotation r1, enum Rotation r2, bool *found = NULL);

// return distance in meters between two locations
float                   get_distance(const struct Location *loc1, const struct Location *loc2);

// return distance in centimeters between two locations
int32_t                 get_distance_cm(const struct Location *loc1, const struct Location *loc2);

// return bearing in centi-degrees between two locations
int32_t                 get_bearing_cd(const struct Location *loc1, const struct Location *loc2);

// see if location is past a line perpendicular to
// the line between point1 and point2. If point1 is
// our previous waypoint and point2 is our target waypoint
// then this function returns true if we have flown past
// the target waypoint
bool        location_passed_point(struct Location & location,
                                  struct Location & point1,
                                  struct Location & point2);

//  extrapolate latitude/longitude given bearing and distance
void        location_update(struct Location *loc, float bearing, float distance);

// extrapolate latitude/longitude given distances north and east
void        location_offset(struct Location *loc, float ofs_north, float ofs_east);



#endif /* LOCATION_H_ */
