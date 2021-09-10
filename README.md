# OMM_Antisaccades
ABRG Oculomotor model extended to make antisaccade decisions.

The antisaccade portion of this model is due to Jennifer Lewis. Jennifer's model is built as an addition to the Oculomotor model of the Gurney group, which is described in the paper [James et. al., Integrating Brain and Biomechanical Models—A New Paradigm for Understanding Neuro-muscular Control, Frontiers 2018](https://www.frontiersin.org/articles/10.3389/fnins.2018.00039/full).

This is a [SpineML](http://spineml.github.io) model, which is opened and run with [SpineCreator](http://spineml.github.io/spinecreator/).

SpineCreator will require you to build BRAHMS, SpineML_PreFlight and you will need to git clone SpineML_2_BRAHMS in your home directory. Follow the instructions to build SpineCreator for [Linux](http://spineml.github.io/spinecreator/sourcelin/) or [Mac](http://spineml.github.io/spinecreator/source/).

You will also need to compile the BRAHMS components from this repository. For this, you will need jsoncpp built and installed in /usr/local. Obtain jsoncpp:
```
mkdir src && pushd src
git clone git@github.com:open-source-parsers/jsoncpp.git
pushd jsoncpp
mkdir build && pushd build
cmake .. && make -j4
sudo make install
popd; popd; popd
```
Now you can build the BRAHMS components
```
cd OMM_Antisaccades/c++/
mkdir build && pushd build
cmake .. && make -j4
popd
```
You then have to install the components in BRAHMS' 'Namespace' - this is a particular directory on your machine (***FIXME: add instructions or modify cmake build to install these automatically***).

You can open the OMM_Antisaccades SpineML project with SpineCreator. The project file to search for from SpineCreator's 'Open project' menu is **spineml/OMM_Antisaccades.proj**.
