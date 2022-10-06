#include <iostream>
#include "td3.hpp"
#include "support.hpp"
#include <stdlib.h>
#include <math.h>       // sin, cos
#include <assert.h>

using namespace std;

using namespace support;

double* extend_array(double* array, int length, int new_size) {
    double* narr = new double[new_size];

    for (int i=0; i<new_size; ++i) {
        if (i<length) {
            narr[i] = array[i];
        }
        else {
            narr[i] = 0;
        }
    }
    delete [] array;

    return narr;

}

double* shrink_array(double* array, int length, int new_size) {
    double* narr = new double[new_size];

    for (int i=0; i<new_size; ++i) {
        narr[i] = array[i];
    }
    delete [] array;

    return narr;

}

double* append_to_array(double element,
                        double* array,
                        int &current_size,
                        int &max_size) {
    if (max_size == current_size) {
        array = extend_array(array, current_size, max_size + 5);
        max_size = max_size + 5;
    }
    array[current_size] = element;
    current_size += 1;

    return array;
}

double* remove_from_array(double* array,
                          int &current_size,
                          int &max_size) {
    current_size -= 1;

    if ((max_size-current_size)>=5) {
        array = shrink_array(array, current_size, max_size - 5);
        max_size-=5;
    }

    return array;

}

bool simulate_projectile(const double magnitude, const double angle,
                         const double simulation_interval,
                         double *targets, int &tot_targets,
                         int *obstacles, int tot_obstacles,
                         double* &telemetry,
                         int &telemetry_current_size,
                         int &telemetry_max_size) {

    bool hit_target, hit_obstacle;
    double v0_x, v0_y, x, y, t;
    double PI = 3.14159265;
    double g = 9.8;

    v0_x = magnitude * cos(angle * PI / 180);
    v0_y = magnitude * sin(angle * PI / 180);

    t = 0;
    x = 0;
    y = 0;

    hit_target = false;
    hit_obstacle = false;
    while (y >= 0 && (! hit_target) && (! hit_obstacle)) {
        telemetry = append_to_array(t, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(x, telemetry, telemetry_current_size, telemetry_max_size);
        telemetry = append_to_array(y, telemetry, telemetry_current_size, telemetry_max_size);

        double * target_coordinates = find_collision(x, y, targets, tot_targets);
        if (target_coordinates != NULL) {
            remove_target(targets, tot_targets, target_coordinates);
            hit_target = true;
        } else if (find_collision(x, y, obstacles, tot_obstacles) != NULL) {
            hit_obstacle = true;
        } else {
            t = t + simulation_interval;
            y = v0_y * t  - 0.5 * g * t * t;
            x = v0_x * t;
        }
    }

    return hit_target;
}

void sort(double *array, const int size) {
    // loop to access each array element
    for (int step = 0; step < (size-1); step+=3) {

        // check if swapping occurs
        bool swapped = 0;

        // loop to compare two elements
        for (int i = 0; i < (size-step-3); i+=3) {

            // compare two array elements
            // change > to < to sort in descending order
            if (array[i] > array[i + 3]) {

                // swapping occurs if elements
                // are not in intended order
                double temp1 = array[i];
                double temp2 = array[i+1];
                double temp3 = array[i+2];

                array[i] = array[i + 3];
                array[i+1] = array[i+4];
                array[i+2] = array[i+5];

                array[i + 3] = temp1;
                array[i + 4] = temp2;
                array[i + 5] = temp3;

                swapped = 1;
            }
        }

        // no swapping means the array is already sorted
        // so no need of further comparison
        if (swapped == 0) {
            break;
        }
    }

}


void merge_telemetry(double **telemetries,
                     int tot_telemetries,
                     int *telemetries_sizes,
                     double* &global_telemetry,
                     int &global_telemetry_current_size,
                     int &global_telemetry_max_size) {
    for (int i=0; i<tot_telemetries; ++i) {
        for (int j=0; j<telemetries_sizes[i]; ++j) {
            global_telemetry = append_to_array(telemetries[i][j], global_telemetry, global_telemetry_current_size, global_telemetry_max_size);
        }
    }
    sort(global_telemetry, global_telemetry_current_size);
}
