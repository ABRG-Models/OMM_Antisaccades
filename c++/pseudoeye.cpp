/*
 * A C++ Brahms component for the Oculomotor model - this one computes
 * a "pseudo eye". It takes input from SCdeep, applies a
 * powercentroid, then it considers whether this centroid exceeds the
 * threshold. If it does, then it updates the output rotations which
 * are fed into the worldDataMaker component.
 *
 * It produces a trajectory for the eye so that the activity is SC_sup
 * is not unrealistically changed by an instantaneous change in World
 * and hence Retina_1 & Retina_2. When the SCdeep goes above its
 * threshold, a simplistic, linear trajectory is precomputed and
 * stored here, then "played out" over 50 ms.
 *
 * Author: Seb James.
 * Date: June-July 2016.
 */

#define COMPONENT_CLASS_STRING "dev/NoTremor/pseudoeye"
#define COMPONENT_CLASS_CPP dev_notremor_pseudoeye_0
#define COMPONENT_RELEASE 0
#define COMPONENT_REVISION 1
#define COMPONENT_ADDITIONAL "Author=Seb James\n" "URL=Not supplied\n"
#define COMPONENT_FLAGS (F_NOT_RATE_CHANGER)

#define OVERLAY_QUICKSTART_PROCESS

#include <brahms-1199.h>
#include <iostream>
#include <math.h>
#include <vector>
#include <stdlib.h>
#include <Eigen/Dense>
#include <utility>

/*!
 * Assume a square neural field. This is the side length. Make sure to
 * update all of these together.
 */
//@{
#define NFS         50
#define NFS_SQUARED 2500
#define NFS_MINUS_1 49
#define NFS_MINUS_2 48
//@}

namespace numeric = std_2009_data_numeric_0;
using namespace std;
using namespace brahms;
using namespace Eigen;

// Use a shorthand for unsigned int in this code file.
typedef unsigned int uint;

/*!
 * This is our Brahms component class declaration.
 */
class COMPONENT_CLASS_CPP : public Process
{
public:
    COMPONENT_CLASS_CPP() {}
    ~COMPONENT_CLASS_CPP() {}

    /*!
     * The framework event function
     */
    Symbol event (Event* event);

private:

    /*!
     * Input - this will be the input from the centroid component.
     */
    //@{
    numeric::Input inputSheet;

    /*!
     * Output the the values in rotX, rotY and rotZ - current
     * rotational state.
     *
     * This is based on a target theoretical eye rotation that the
     * centroid (in inputSheet) encodes, as long as it is above the
     * threshold.
     */
    numeric::Output rotationsOut;

    /*!
     * The centroid output as a sheet.
     */
    numeric::Output centroid;

    /*!
     * Current rotational state. Initialised to 0
     */
    //@{
    double rotX;
    double rotY;
    double rotZ;
    //@}

    /*!
     * Target rotational trajectory as encoded by SC deep. Initialised to 0
     */
    //@{
    vector<double> tRotX;
    vector<double> tRotY;
    vector<double> tRotZ;
    //@}

    /*!
     * Is this the first saccade? Due to the difficulty of handling
     * the edge effects in the model, which make the theoretical
     * this->retmap inaccurate, we allow the first saccade to occur
     * and then allow no further saccades. Initialised to true.
     */
    bool firstsaccade;

    /*!
     * The current fixed, BRAHMS timestep in milliseconds. This is
     * obtained at runtime, as it is defined in the BRAHMS system xml
     * file.
     */
    float dt_ms;

    /*!
     * What power to give the power centroid.
     */
    double power_degree;

    /*!
     * The threshold above which the rotations (rotX-rotZ) will be
     * updated.
     */
    double threshold;

    /*!
     * A period during which the output cannot be set.
     */
    uint settleTime;

    /*!
     * Number of neurons in a cortical sheet. Should be the square of
     * an integer.
     */
    int neuronsPerPopulation;

    /*!
     * Theoretical retinotopic mapping. See RetinotopicMapping.ipynb
     * for a python implementation.
     */
    std::pair<double,double> retmap (int r, int phi);

    /*!
     * Get max value from input.
     */
    double getMax (double* input);

    /*!
     * Apply the power centroid
     */
    void apply_centroid (double* input);

    /*!
     * Used in apply_centroid
     */
    //@{
    Array<double, NFS, NFS> X;
    Array<double, NFS, NFS> Y;
    double sigma_r_dot_a_x;
    double sigma_r_dot_a_y;
    double sigma_a;
    uint centroid_x;
    uint centroid_y;
    //@}

    /*!
     * Output file stream for the saccsim_side.log file which we
     * produce to look like the one generated from the real
     * biomechanical eye model.
     */
    //std::ofstream ss;
    FILE* ss;

    std::string saveDirPath;

public:
  /*!
   * Fix because I have a statically allocated Eigen object as a
   * member of this class. See
   * http://eigen.tuxfamily.org/dox-devel/group__TopicStructHavingEigenMembers.html
   * for details.
   */
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#define PI 3.14159265
pair<double,double>
COMPONENT_CLASS_CPP::retmap (int r, int phi)
{
    double E2 = 2.5; // radial angle at which neural density has halved
    double nfs = 50; // 50x50 grid.
    double fieldOfView = 61; // 0 and +/- 30 degrees
    // The Magnification factor
    double Mf = nfs/(E2*log( ((fieldOfView/2) / E2+1) ));

    double thetax = E2*(-1+exp(double(r)/(Mf*E2)))*cos(phi*2*PI/nfs);
    double thetay = E2*(-1+exp(double(r)/(Mf*E2)))*sin(phi*2*PI/nfs);

    return std::make_pair(thetax,thetay);
}

/*!
 * Eigen based method for finding the max of a sheet.
 */
double
COMPONENT_CLASS_CPP::getMax (double* input)
{
    // Get input as an array without copying the data:
    Map<Array<double, NFS, NFS> > sheet(input,NFS,NFS);

    double themax =  sheet.maxCoeff();
    //bout << "The max is " << themax << D_INFO;
    return themax;
}

/*!
 * Eigen based method for centroiding.
 */
void
COMPONENT_CLASS_CPP::apply_centroid (double* input)
{
    // Wrap the output in an array, ready for output
    Map<Array<double, NFS, NFS> > the_centroid((double*)this->centroid.getContent(),NFS,NFS);
    // Zero out the result of the last timestep:
    the_centroid.setZero();

    // Get input as an array without copying the data:
    Map<Array<double, NFS, NFS> > sheet(input,NFS,NFS);

    sigma_r_dot_a_x = (this->X*sheet).pow(this->power_degree).sum();
    sigma_r_dot_a_y = (this->Y*sheet).pow(this->power_degree).sum();
    this->sigma_a = sheet.pow(this->power_degree).sum();
    this->centroid_x = (uint)round(pow(sigma_r_dot_a_x/sigma_a, 1.0/this->power_degree));
    this->centroid_y = (uint)round(pow(sigma_r_dot_a_y/sigma_a, 1.0/this->power_degree));
    //bout << "indices " << this->centroid_x << "," << this->centroid_y << " for the_centroid" << " with sigma_a=" << sigma_a << D_INFO;
    // Decrement ops here as Eigen arrays count from 0.
    if (this->centroid_x == 0 || this->centroid_y == 0) {
        // sigma_r_dot_a_x or y must've been 0.
        this->centroid_x = 25;
        this->centroid_y = 25;
    }
    uint cx = this->centroid_x-1;
    uint cy = this->centroid_y-1;
    if (cx < 0 || cx >= NFS || cy < 0 || cy >= NFS) {
        berr << "BAD indices " << cx << "," << cy << " for the_centroid";
    } else {
        the_centroid (this->centroid_x-1, this->centroid_y-1) = this->sigma_a;
    }
}

/*!
 * This is the implementation of our component class's event method
 */
Symbol COMPONENT_CLASS_CPP::event(Event* event)
{
    switch (event->type) {

    case EVENT_STATE_SET: // Get state from the node's XML.
    {
        bout << "EVENT_STATE_SET." << D_INFO;
        // extract DataML
        EventStateSet* data = (EventStateSet*) event->data;
        XMLNode xmlNode(data->state);
        DataMLNode nodeState(&xmlNode);

        // Generate X, Y and ones
        VectorXd x = VectorXd::LinSpaced(NFS,1,NFS);
        this->X = x.rowwise().replicate(NFS).array();
        this->Y = x.transpose().colwise().replicate(NFS).array();

        this->dt_ms = 1000 * static_cast<float> (this->time->sampleRate.den) / static_cast<float> (this->time->sampleRate.num);

        // The path for storing saccsim_side.log
        this->saveDirPath = nodeState.getField ("output_data_path").getSTRING();
        string cmd = "mkdir -p " + this->saveDirPath;
        int srtn = system (cmd.c_str());
        if (srtn) {
            berr << "Error creating output data directory '" << this->saveDirPath << "'";
        }
        string fpath = this->saveDirPath + "/saccsim_side.log";
        this->ss = fopen (fpath.c_str(), "w");
        fprintf (this->ss, "BST,AMR,ALR,AIR,ASR,ASO,AIO,RX,RY,RZ,RXu,RYu,RZu,SST\n");

        this->firstsaccade = true;

        // The number of neurons per population in the OM model (2500 or 50x50)
        this->neuronsPerPopulation = NFS * NFS;

        // Get params from the SystemML:
        this->threshold = nodeState.getField ("threshold").getDOUBLE();
        this->power_degree = nodeState.getField ("power_degree").getDOUBLE();
        this->settleTime = nodeState.getField ("settleTime").getUINT32();

        rotX = 0;
        rotY = 0;
        rotZ = 0;

        return C_OK;
    }

    case EVENT_INIT_CONNECT:
    {
        bout << "EVENT_INIT_CONNECT." << D_INFO;

        if (event->flags & F_FIRST_CALL)
        {
            bout << "EVENT_INIT_CONNECT, F_FIRST_CALL." << D_INFO;

            this->centroid.setName("centroid");
            this->centroid.create(hComponent);
            this->centroid.setStructure(TYPE_DOUBLE | TYPE_REAL, Dims(this->neuronsPerPopulation).cdims());
            this->rotationsOut.setName("rotationsOut");
            this->rotationsOut.create(hComponent);
            this->rotationsOut.setStructure(TYPE_DOUBLE | TYPE_REAL, Dims(6).cdims()); // 3 rotns and 3 rotn speeds

            this->inputSheet.attach (hComponent, "inputSheet");
        }

        // on last call
        if (event->flags & F_LAST_CALL)
        {
            bout << "EVENT_INIT_CONNECT, F_LAST_CALL." << D_INFO;
            // Do anything that has to be done on the last call.
            this->inputSheet.validateStructure (TYPE_REAL|TYPE_DOUBLE, Dims(this->neuronsPerPopulation).cdims());
        }

        // ok, INIT_CONNECT event serviced.
        return C_OK;
    }

    case EVENT_INIT_POSTCONNECT:
    {
        return C_OK;
    }

    case EVENT_RUN_SERVICE:
    {
        // current brahms simulation time
        double simtime_ms = float(this->time->now) * this->dt_ms;

        // Apply the centroid - note that this populates one of the outputs
        // and so must be called every time EVENT_RUN_SERVICE occurs
        this->apply_centroid ((double*)this->inputSheet.getContent());

	// Find max of this->inputSheet.getContent()
        double themax = this->getMax((double*)this->inputSheet.getContent());

        // Check threshold; update rotations if necessary
        //bout << "time=" << simtime_ms << " > settleTime=" << this->settleTime << "?" << D_INFO;
        //bout << "AND sigma_a=" << this->sigma_a << " > threshold=" << this->threshold << "?" << D_INFO;
        if (simtime_ms > this->settleTime && themax > this->threshold && this->tRotX.empty() && firstsaccade == true) {

            // Update tRotX,tRotY (tRotZ stays 0)
            pair<double,double> thx_thy = this->retmap (this->centroid_y, this->centroid_x);
            // Now create vectors for the trajectory
            this->tRotX.clear();
            this->tRotY.clear();
            this->tRotZ.clear();
            double xstep = (thx_thy.first-rotX)/50.0;
            double ystep = (thx_thy.second-rotY)/50.0;
            bout << "Setting up new tRotX target: " << thx_thy.first << " and tRotY target:" << thx_thy.second << D_INFO;
            for (unsigned int i=0; i<50; ++i) {
                this->tRotX.push_back (thx_thy.first - i*xstep);
                this->tRotY.push_back (thx_thy.second - i*ystep);
                this->tRotZ.push_back (0);
            }
            // We've now precomputed our first saccade
            this->firstsaccade = false;
        }

        // Update rotX,rotY,rotZ if there's a trajectory to follow
        if (!this->tRotX.empty()) {
            this->rotX = this->tRotX.back();
            bout << "tRotX is not empty, set rotX to " << this->rotX << D_INFO;
            this->tRotX.pop_back();
            this->rotY = this->tRotY.back();
            this->tRotY.pop_back();
            this->rotZ = this->tRotZ.back();
            this->tRotZ.pop_back();
        }

        // On every timestep, put rotX,rotY,rotZ into rotationsOut.
        double rOutArr[6];
        double* rOut = &rOutArr[0];
        *rOut++ = rotX;
        *rOut++ = rotY;
        *rOut++ = rotZ;
        *rOut++ = 0.0; // RotX speed (N/A)
        *rOut++ = 0.0; // RotY speed (N/A)
        *rOut = 0.0; // RotZ speed (N/A)
        this->rotationsOut.setContent (&rOutArr, 0, 6 * sizeof(DOUBLE));

        // Output into saccsim_side.log
        double simtime_s = simtime_ms / 1000;
        fprintf (this->ss, "%f,0,0,0,0,0,0,%f,%f,%f,%f\n",
                 simtime_s,rotX,rotY,rotZ,simtime_s);
        fflush (this->ss);

        // ok, RUN_SERVICE event serviced.
        return C_OK;
    }

    case EVENT_RUN_STOP:
    {
        fclose (this->ss);
        return C_OK;
    }

    } // switch (event->type)

    //	if we serviced the event, we returned C_OK if we didn't, we
    //	should return S_NULL to indicate this.
    return S_NULL;
}

// Here at the end, include the second part of the overlay (it knows
// you've included it once already).
#include "brahms-1199.h"
