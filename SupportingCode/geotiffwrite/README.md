This program can write a 2D or 3D array to a single or multi-band GeoTIFF file. For more information and examples, please visit:  
[http://research.baycrest.org/~jimmy/geotiffwrite](http://research.baycrest.org/~jimmy/geotiffwrite)  
or type: help geotiffwrite

MATLAB's Mapping Toolbox only provides a "geotiffread" function, but it does not have a "geotiffwrite" function (Note). This is the MATLAB program to write a 2D or 3D array to a single or multi-band GeoTIFF file, where data can be either 1-bit monochrome data (i.e. binary or bilevel), 8-bit unsigned integer, 16-bit signed / unsigned integer, 32-bit signed integer, or 32-bit floating point.

Note: Starting from version R2011a, MATLAB also provides its own "geotiffwrite" function  
([http://www.mathworks.com/help/toolbox/map/rn/bsq4us7-1.html#bsu1ro5-1](http://www.mathworks.com/help/toolbox/map/rn/bsq4us7-1.html#bsu1ro5-1)).

This program is based on GeoTIFF Format Specification under:  
[http://www.remotesensing.org/geotiff/spec/geotiffhome.html](http://www.remotesensing.org/geotiff/spec/geotiffhome.html)  
or [http://download.osgeo.org/geotiff/spec](http://download.osgeo.org/geotiff/spec)

It does not need MATLAB's Mapping Toolbox, or any other library.

Usage:  
[option bbox] = make\_option([bbox]);  
geotiffwrite(filename, bbox, image, [bit\_depth, option]);

For more information on arguments and detail examples, please visit:  
[http://research.baycrest.org/~jimmy/geotiffwrite](http://research.baycrest.org/~jimmy/geotiffwrite)  
or type: help geotiffwrite