#!/bin/bash

for i in *.xml; do
    echo ----------------------- >> ../xmldiff.txt;
    echo : >> ../xmldiff.txt;
    echo -------------------- >> ../xmldiff.txt;
    xmldiff -p  ../spineml/ >> ../xmldiff.txt;
done
