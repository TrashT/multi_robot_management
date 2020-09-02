Setting up the server:
 1. Copy the 'html' folder to /var/www
 2. Use npm to install any packages missing in html/javascript
 3. Run Apache HTTP Server
 4. Install the turtlebot3_fleet_manager package

Starting the server:
 Run html/javascript/server.js with node.js and include the name of your network as a parameter (e.g. node server.js eno1)

Before adding a robot:
Make sure the robot is switched on and connected to the internet, and that its time is synchronized with the server time.

Changing the map:
 1. Replace the files in html/map
 2. Update MAP_ORIGIN in html/javascript/scripts/workspace provider.js to match the origin in officeMap.yaml
 3. From officeMap.pgm, create an "office map cool-looking.png" file (try to trim it as much as possible, note step 4), and (re)place it in html/img
 4. Update MAP_TRANSLATE in html/javascript/scripts/client.js to reflect how many pixels you trimmed, from the left and the top respectively, when creating the   	 new .png file (set it to (0, 0) if you didn't trim anything).
 5. Change the dimensions of the "map" canvas element in html/index.html to match the dimensions of the new .png file.
 6. Analogously, change the dimensions of #mapDiv in html/styles.css
