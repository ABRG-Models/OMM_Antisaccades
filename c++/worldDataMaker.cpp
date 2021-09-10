/*
 * A C++ Brahms component for the Oculomotor model world data maker.
 *
 * Intention for this is to accept data for luminances via the Brahms
 * state setup and then process both output to the Oculomotor brain
 * model, as well as input from the rotations generated by the
 * biomechanical eye model.
 *
 * This component works with a world reference frame, and an eye
 * reference frame, the centre of which is the fovea. Input luminances
 * are provided with spatial coordinates in the world reference frame
 * and time coordinates.
 *
 * An offset between the eye frame and the world frame is maintained,
 * allowing the location of luminances to be calculated in both
 * frames.
 *
 * At each timestep, the current response of the eye's light detecting
 * neurons is determined and mapped onto the 50x50 square cortical
 * map. This square layout is used throughout the OM brain model for
 * neural populations.
 *
 * This component is based on the worldDataMaker.m code, which is the
 * work of Alex Blenkinsop and Alex Cope.
 *
 * Author: Seb James.
 * Date: Sept 2014.
 */

#define COMPONENT_CLASS_STRING "dev/NoTremor/worldDataMaker"
#define COMPONENT_CLASS_CPP dev_notremor_worlddatamaker_0
#define COMPONENT_RELEASE 0
#define COMPONENT_REVISION 1
#define COMPONENT_ADDITIONAL "Author=Seb James\n" "URL=Not supplied\n"
#define COMPONENT_FLAGS (F_NOT_RATE_CHANGER)

#define OVERLAY_QUICKSTART_PROCESS

#include <brahms-1199.h>
#include <iostream>
#include "luminance.h"
#include "worldframe.h"
#include "eyeframe.h"

#include <json/json.h>

#ifdef __WIN__
// Windows specific includes/apologies here.
#else
// Linux specific includes here.
#endif

namespace numeric = std_2009_data_numeric_0;
using namespace std;
using namespace brahms;
using namespace wdm;

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
     * Output port(s). This will be a cortico-centric map of the
     * current luminance - a 50x50 map.
     */
    numeric::Output corticalSheet;

#ifdef SEPARATE_ROT_INPUTS
    /*!
     * Input ports. see brahms-x.x.x/components/std.h/data_numeric.h
     * for the declaration of the struct Input.
     */
    //@{
    numeric::Input rotX;
    numeric::Input rotY;
    numeric::Input rotZ;
    //@}
#else
    /*!
     * Input rotations - rotX, rotY, rotZ (Euler rotations) and rotXu,
     * rotYu, rotZu (Euler rotational speeds). Same format as
     * rotationsOut in the saccsim component; 3 rotations, 3 angular
     * velocities as 6 doubles.
     */
    numeric::Input rotationsIn;
#endif

    /*!
     * The directory into which cortical sheets will be saved.
     */
    string saveDirPath;

    /*!
     * The current fixed, BRAHMS timestep. This is obtained at
     * runtime, as it is defined in the BRAHMS system xml file.
     */
    float dt;

    /*!
     * Number of neurons in a cortical sheet. Should be the square of
     * an integer.
     */
    int neuronsPerPopulation;

    /*!
     * Data about the world as it appears in the eye's frame of
     * reference.
     */
    wdm::EyeFrame eye;

    /*!
     * Data about the world. Holds the current luminance map, along
     * with the information about where lights are going to appear in
     * the world during the experiment, and when they turn on and off.
     */
    wdm::WorldFrame world;

private:
    /*!
     * Open and read a simple JSON file which contains the parameters
     * for the luminances for the experimental run. This information
     * COULD be encoded in the BRAHMS sys.xml file, but that would be
     * I would have to hack the already-hacked SpineML_2_BRAHMS code
     * for each experimental run, and it's just nicer to write and
     * read config in JSON format.
     *
     * Sample JSON:
     * {
     *   // Single target luminance example. "luminances" is an array.
     *   "luminances": [
     *     // This is the fixation luminance:
     *     {
     *       "shape":"cross",
     *       "thetaX":0,
     *       "thetaY":0,
     *       "widthThetaX":6,
     *       "widthThetaY":2,
     *       "luminance":0.5,
     *       "timeOn":0.0,
     *       "timeOff":0.050
     *     },
     *     // Here's the target luminance (because it comes on after
     *     // the previous one, see the timeOn element):
     *     {
     *       "shape":"cross",
     *       "thetaX":0,
     *       "thetaY":10,
     *       "widthThetaX":6,
     *       "widthThetaY":2,
     *       "luminance":0.5,
     *       "timeOn":0.070,
     *       "timeOff":0.600
     *     }
     *   ]
     * }
     *
     * A file containing JSON similar to the above should be written
     * into a file called luminances.json in the Oculomotor SpineML
     * model directory.
     */
    void readLuminanceFile (void);
};

/*!
 * The "read the JSON luminance config file" method implementation.
 */
#define LUMINANCES_FILE "../model/luminances.json"
void COMPONENT_CLASS_CPP::readLuminanceFile (void)
{
    // This will read a file called luminances.json. The present
    // working directory will be your SpineML_2_BRAHMS
    // output_file_location/run/. Place luminances.json into your
    // model directory and this will work:

    // Test for existence of the file.
    ifstream jsonfile_test;
    int srtn = system ("pwd");
    if (srtn) {
        bout << "system call returned " << srtn << D_INFO;
    }
    jsonfile_test.open (LUMINANCES_FILE, ios::in);
    if (jsonfile_test.is_open()) {
        // Good, file exists.
        bout << "Good, luminances.json file in model directory exists." << D_INFO;
        jsonfile_test.close();
    } else {
        berr << "luminances.json file not found.";
        return;
    }

    // Parse the JSON
    ifstream jsonfile (LUMINANCES_FILE, ifstream::binary);
    Json::Value root;
    string errs;
    Json::CharReaderBuilder rbuilder;
    rbuilder["collectComments"] = false;
    bool parsingSuccessful = Json::parseFromStream (rbuilder, jsonfile, &root, &errs);
    if (!parsingSuccessful) {
        // report to the user the failure and their locations in the document.
        berr << "Failed to parse JSON luminances: " << errs;
        return;
    }

    const Json::Value plugins = root["luminances"];
    for (int index = 0; index < plugins.size(); ++index) {  // Iterates over the sequence elements.
        Json::Value v = plugins[index];
        // args for Luminance are: (thX, thY, crosswidth, barthickness, brightness, timeOn, timeOff)
        if (v.get ("shape", "cross").asString() == "cross") {
            CrossLuminance cross_lum (v.get ("thetaX", 0.0).asInt(),
                                      v.get ("thetaY", 0.0).asInt(),
                                      v.get ("widthThetaX", 0.0).asInt(),
                                      v.get ("widthThetaY", 0.0).asInt(),
                                      v.get ("luminance", 0.0).asFloat(),
                                      v.get ("timeOn", 0.0).asFloat(),
                                      v.get ("timeOff", 0.0).asFloat());
            this->world.luminanceSeries.push_back (cross_lum);
        } else {
            Luminance rect_lum (v.get ("thetaX", 0.0).asInt(),
                                v.get ("thetaY", 0.0).asInt(),
                                v.get ("widthThetaX", 0.0).asInt(),
                                v.get ("widthThetaY", 0.0).asInt(),
                                v.get ("luminance", 0.0).asFloat(),
                                v.get ("timeOn", 0.0).asFloat(),
                                v.get ("timeOff", 0.0).asFloat());
            this->world.luminanceSeries.push_back (rect_lum);
        }
        bout << "Output " << v.get ("shape", 0.0).asString()
             << "luminance with params:" << D_INFO;
        bout << "thetaX:" << v.get ("thetaX", 0.0).asInt()
             << " thetaY:" << v.get ("thetaY", 0.0).asInt()
             << " widthThetaX:" << v.get ("widthThetaX", 0.0).asInt()
             << " widthThetaY:" << v.get ("widthThetaY", 0.0).asInt()
             << " luminance:" << v.get ("luminance", 0.0).asFloat()
             << " timeOn:" << v.get ("timeOn", 0.0).asFloat()
             << " timeOff:" << v.get ("timeOff", 0.0).asFloat() << D_INFO;
    }
}

/*!
 * This is the implementation of our component class's event method
 */
Symbol COMPONENT_CLASS_CPP::event(Event* event)
{
    // Could do with a macro to show what event is caught:
    //bout << "Brahms event caught: " << event->type << D_INFO;

    switch (event->type) {

    case EVENT_STATE_SET: // Get state from the node's XML.
    {
        bout << "EVENT_STATE_SET." << D_INFO;
        // extract DataML
        EventStateSet* data = (EventStateSet*) event->data;
        XMLNode xmlNode(data->state);
        DataMLNode nodeState(&xmlNode);

        // Get size of our fixed timestep IN SECONDS (to match saccsim time)
        this->dt = static_cast<float> (this->time->sampleRate.den) / static_cast<float> (this->time->sampleRate.num);

        // The path for storing the output data csv.
        this->saveDirPath = nodeState.getField ("output_data_path").getSTRING() + "/wdm";
        string cmd = "mkdir -p " + this->saveDirPath;
        int srtn = system (cmd.c_str());
        if (srtn) {
            berr << "Error creating output data directory '" << this->saveDirPath << "'";
        }

        // The number of neurons per population in the OM model (2500 or 50x50)
        this->neuronsPerPopulation = nodeState.getField ("neuronsPerPopulation").getINT32();
        // Test it's a square.

        return C_OK;
    }

    case EVENT_INIT_CONNECT:
    {
        bout << "EVENT_INIT_CONNECT." << D_INFO;

        if (event->flags & F_FIRST_CALL)
        {
            bout << "EVENT_INIT_CONNECT, F_FIRST_CALL." << D_INFO;

            this->corticalSheet.setName("corticalSheet");
            this->corticalSheet.create(hComponent);
            this->corticalSheet.setStructure(TYPE_DOUBLE | TYPE_REAL, Dims(this->neuronsPerPopulation).cdims());

            // Attaching the six inputs (NB! We're attaching the same input to each one here!)
            // How does system know to attach sup. rect input to this? Is it by order?
#ifdef SEPARATE_ROT_INPUTS
            if (this->rotX.tryAttach (hComponent, "rotX")) {
                bout << "attached the input for the rotX component." << D_INFO;
            } else {
                berr << "Failed to attach the input for the rotX component.";
            }
            if (this->rotY.tryAttach (hComponent, "rotY")) {
                bout << "attached the input for the rotY component." << D_INFO;
            } else {
                berr << "Failed to attach the input for the rotY component.";
            }
            if (this->rotZ.tryAttach (hComponent, "rotZ")) {
                bout << "attached the input for the rotZ component." << D_INFO;
            } else {
                berr << "Failed to attach the input for the rotZ component.";
            }
#else
            //if (this->rotationsIn.tryAttach (hComponent, "rotationsIn")) {
            //    bout << "attached the input for the rotations component." << D_INFO;
            //} else {
            //    berr << "Failed to attach the input for the rotations component.";
            //}
            this->rotationsIn.attach (hComponent, "rotationsIn");
            this->rotationsIn.validateStructure (TYPE_REAL|TYPE_DOUBLE, Dims(6).cdims());
#endif
        }

        // on last call
        if (event->flags & F_LAST_CALL)
        {
            bout << "EVENT_INIT_CONNECT, F_LAST_CALL." << D_INFO;
            // Do anything that has to be done on the last call.
        }

        // ok, INIT_CONNECT event serviced.
        return C_OK;
    }

    case EVENT_INIT_POSTCONNECT:
    {
        // After the connecting of connections, we'll set up our world
        // data. This will use information passed in from the DataML
        // parameters in the component state, sometimes called
        // sys.xml, but for now, and for testing, I'll hard-code.
        // Update on input data: If I read input from sys.xml, then I
        // have to write code in SpineML_2_BRAHMS to generate that
        // extra data, so I'm not going to do it that way. Instead,
        // I'll have this component read from a special file to get
        // the target information. The format will be: int fixation x,
        // int fixation y, int fixation width, int fixation barsize,
        // double fixation luminance, double fix start, double fix
        // end. In binary format. Then repeat similar lines for
        // target1, target 2 etc up to as many targets as desired.

        // Init world again with 0 BG luminance.
        this->world.init (0.0);

        // Read in the luminance file (this method adds luminances to
        // this->world).
        this->readLuminanceFile();

        // Initialise the eye
        this->eye.setDistanceToScreen (this->world.distanceToScreen);
        this->eye.setOffset (0, 0, 0);

        return C_OK;
    }

    case EVENT_RUN_SERVICE:
    {
        // current brahms simulation time
        double simtime = float(this->time->now) * this->dt;
        this->world.setLuminanceThetaMap (simtime); // Set up the map for time=X s.

        // TO make a copy of the current X rotation:
#ifdef SEPARATE_ROT_INPUTS
        double curRotX = *((double*)this->rotX.getContent());
        double curRotY = *((double*)this->rotY.getContent());
        double curRotZ = *((double*)this->rotZ.getContent());
#else
        //copy the 6 doubles from this->rotationsIn into some sort of container.
        double* rotComponent = (double*)this->rotationsIn.getContent();
        double curRotX = *rotComponent;
        double curRotY = *(++rotComponent);
        double curRotZ = *(++rotComponent);
#endif
        bout << "WDM: rotationsOut X/Y/Z: "
             << curRotX << "," << curRotY << "," << curRotZ << D_INFO;

        // Output eye's offset into a file so we can track what's going on
        stringstream iss;
        iss << this->saveDirPath << "/info" << this->time->now << ".dat";
        ofstream info;
        info.open (iss.str().c_str(), ios::out|ios::trunc);

        info << "X/Y/Z from curRotX etc: " << curRotX << "," << curRotY << "," << curRotZ << endl;

        // Calculate new luminances here.
        this->eye.setOffset (curRotX, curRotY, curRotZ);

        info << this->time->now << "ms: RotX:" << this->eye.getRotX() << " RotY:" << this->eye.getRotY() << " RotZ:" << this->eye.getRotZ() << endl; // etc
        info.close();

        this->eye.setEyeField (this->world.luminanceCoords);

        // Save the cortical sheet so we can make movies with it.
        stringstream cmss;
        cmss << this->saveDirPath << "/cortmap" << this->time->now << ".dat";
        this->eye.saveCorticalSheet (cmss.str());

        // Write out luminances to the model
        this->corticalSheet.setContent (this->eye.getCorticalSheetData(), 0, this->neuronsPerPopulation * sizeof(DOUBLE));

        // ok, RUN_SERVICE event serviced.
        return C_OK;
    }

    case EVENT_RUN_STOP:
    {
        bout << "EVENT_RUN_STOP" << D_INFO;
        // Do any cleanup necessary here.
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
