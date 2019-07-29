#!/bin/bash
for file in ./*.pdf
do
    pdfcrop $file
done
