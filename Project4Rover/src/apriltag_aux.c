//Auxilliary Functions extracted from apriltag.c


#include <math.h>
#include <stdio.h>
 #include <stdbool.h>
#include "debug_print.h"
#include "apriltag_pose.h"
#include "homography.h"
#include "matd.h"



static int quad_update_homographies(struct quad *quad);
static matd_t* homography_compute2(double c[4][4]);

int update_homography(apriltag_detection_t *mydet){
    struct quad my_quad={0};
    for (size_t k = 0; k < 4; k++)
    {
        my_quad.p[k][0] = mydet->p[k][0];
        my_quad.p[k][1] = mydet->p[k][1];

    }
    //printf("Calculating homography\n");
    
    int ret = quad_update_homographies(&my_quad);
    //printf("Copying homography\n");
    if (ret == 0)
    {
        if (mydet->H)
        {
            matd_destroy(mydet->H);
        }
        mydet->H = matd_copy(my_quad.H);
        matd_destroy(my_quad.H);
        matd_destroy(my_quad.Hinv);
        
    } else{
        printf("Failed to compute homography\n");
    }
    return ret;
}

static int quad_update_homographies(struct quad *quad)
{
    //zarray_t *correspondences = zarray_create(sizeof(float[4]));

    double corr_arr[4][4];
    //printf("Copying data from quad\n");
    for (int i = 0; i < 4; i++) {
        corr_arr[i][0] = (i==0 || i==3) ? -1 : 1;
        corr_arr[i][1] = (i==0 || i==1) ? -1 : 1;
        corr_arr[i][2] = quad->p[i][0];
        corr_arr[i][3] = quad->p[i][1];
    }

    if (quad->H)
        matd_destroy(quad->H);
    if (quad->Hinv)
        matd_destroy(quad->Hinv);

    // XXX Tunable
    //printf("Inner Compute\n");
    quad->H = homography_compute2(corr_arr);
    if (quad->H != NULL) {
        quad->Hinv = matd_inverse(quad->H);
        if (quad->Hinv != NULL) {
	    // Success!
            
            return 0;
        }
        matd_destroy(quad->H);
        quad->H = NULL;
    }
    return -1;
}

static matd_t* homography_compute2(double c[4][4]) {
    double A[] =  {
            c[0][0], c[0][1], 1,       0,       0, 0, -c[0][0]*c[0][2], -c[0][1]*c[0][2], c[0][2],
                  0,       0, 0, c[0][0], c[0][1], 1, -c[0][0]*c[0][3], -c[0][1]*c[0][3], c[0][3],
            c[1][0], c[1][1], 1,       0,       0, 0, -c[1][0]*c[1][2], -c[1][1]*c[1][2], c[1][2],
                  0,       0, 0, c[1][0], c[1][1], 1, -c[1][0]*c[1][3], -c[1][1]*c[1][3], c[1][3],
            c[2][0], c[2][1], 1,       0,       0, 0, -c[2][0]*c[2][2], -c[2][1]*c[2][2], c[2][2],
                  0,       0, 0, c[2][0], c[2][1], 1, -c[2][0]*c[2][3], -c[2][1]*c[2][3], c[2][3],
            c[3][0], c[3][1], 1,       0,       0, 0, -c[3][0]*c[3][2], -c[3][1]*c[3][2], c[3][2],
                  0,       0, 0, c[3][0], c[3][1], 1, -c[3][0]*c[3][3], -c[3][1]*c[3][3], c[3][3],
    };

    double epsilon = 1e-10;

    //matd_print(A,"%lf");

    // Eliminate.
    for (int col = 0; col < 8; col++) {
        // Find best row to swap with.
        double max_val = 0;
        int max_val_idx = -1;
        for (int row = col; row < 8; row++) {
            double val = fabs(A[row*9 + col]);
            if (val > max_val) {
                max_val = val;
                max_val_idx = row;
            }
        }

        if (max_val_idx < 0) {
            return NULL;
        }

        if (max_val < epsilon) {
            debug_print("WRN: Matrix is singular.\n");
            return NULL;
        }

        // Swap to get best row.
        if (max_val_idx != col) {
            for (int i = col; i < 9; i++) {
                double tmp = A[col*9 + i];
                A[col*9 + i] = A[max_val_idx*9 + i];
                A[max_val_idx*9 + i] = tmp;
            }
        }

        // Do eliminate.
        for (int i = col + 1; i < 8; i++) {
            double f = A[i*9 + col]/A[col*9 + col];
            A[i*9 + col] = 0;
            for (int j = col + 1; j < 9; j++) {
                A[i*9 + j] -= f*A[col*9 + j];
            }
        }
    }

    // Back solve.
    for (int col = 7; col >=0; col--) {
        double sum = 0;
        for (int i = col + 1; i < 8; i++) {
            sum += A[col*9 + i]*A[i*9 + 8];
        }
        A[col*9 + 8] = (A[col*9 + 8] - sum)/A[col*9 + col];
    }
    return matd_create_data(3, 3, (double[]) { A[8], A[17], A[26], A[35], A[44], A[53], A[62], A[71], 1 });
}

