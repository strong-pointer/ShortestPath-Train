# ShortestPath-Train
Demonstration of Dijkstra's Shortest Path Algorithm on an image with restraints

This program is meant to run with the included image, and it usually runs ~10 seconds (with the provided restraints and ppm), depending on your machine.

It is meant to practice maximizing the running time as much as possible. The program features Dijkstra's Algorithm, bit shifting, and many other thoughtful "workarounds" to speed processes up.<br>
A config file is required alongside an image to create a path. I have included a sample config.txt and railway.ppm file for use.

Config files are formatted as follows:<br><pre>
pictureFileName
startingPointX startingPointY
goalX 	       goalY
passenger1X    passenger1Y
passenger2X    passenger2Y
passenger3X    passenger3Y
passenger4X    passenger4Y
maximumPixelColor (when summed)
minEuclidianDistanceBetweenPixels
maxEuclidianDistanceBetweenPixels
maxTurningDegrees
D
</pre>

Run the program in this format:
<pre>./[executableName] configFile</pre>
