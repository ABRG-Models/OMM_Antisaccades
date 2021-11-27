#!/bin/bash

for i in *.xml; do
    echo "-----------------------" >> ../xmldiff.txt;
    echo "${i}: " >> ../xmldiff.txt;
    echo "--------------------" >> ../xmldiff.txt;
    xmldiff -p  ${i} ../spineml/${i} >> ../xmldiff.txt;
done
