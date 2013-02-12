ofxAruco
========

openFrameworks addon for the AR library ArUco: 
 - http://www.uco.es/investiga/grupos/ava/node/26

Much simpler than other marker based AR libraries.

It's main features:

   * Detect markers with a single line of C++ code
   * Detection of AR boards (markers composed by several markers)
   * Requires only OpenCv (>=2.1)
   * Up to to 1024 different markers
   * Trivial integration with OpenGL and OGRE
   * Fast, reliable and cross-platform because relies on OpenCv
   * Examples that will help you to get running your AR application in less than 5 minutes
   * BSD licence

Notes:

* begin/beginBoard set openGL to use normalized cordinates from -1,1 with 0,0 
in the center of the marker or board with positive y upwards
