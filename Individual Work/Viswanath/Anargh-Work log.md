# Software Development Project - Weekly work progress
(Author - Anargh)
-----------------------------------------------------------------------------------------------
# Analysis done on cockpit (1st week results) (25.10.2019)

-----------------------------------------------------------------------------------------------

* First of all, managed to install the cockpit system on to the system using the command
'''bash
sudo apt-get install cockpit
'''

* Tried to analyze the working by viewing the code but because of unfamiliarity with Java script --> not much gained

* Went through the videos of cockpit. The makers of videos illustrated the installation of application but not much information on how the applications were made.

* After discussion with Deebul, got a lead towards installing plug-in through Starter kit module on Cockpit blogs instead of creating applications.

* Went through the documentation for Starter kit for cockpit on https://cockpit-project.org/blog/cockpit-starter-kit.html to understand how to create own pages and plug-ins for cockpit.

* Separately installed Starter kit module by following instructions on https://github.com/cockpit-project/starter-kit
-- (Had problems  initially because npm wasn't installed on the system as suggested on the link)

* After this, to understand how the packages work, copied the files of pinger example into the folder of Cockpit placed at the location usr/share/cockpit into a new folder named Test.

* The pinger example allowed the realization that there are 3 essential components required to create a new page -->
1. manifest.json file -- so as to help cockpit realize that this has to be added no matter wherever this file is on the system (Not necessary to be placed in the cockpit files folder)
2. <plugin>.html -- to design the html page
3. <plugin>.js  -- to create functionalities on the page in java script

* Tweaked the code of pinger to create a new menu item. Added new items in .js file which didn't work as expected.

* Looked into rosbridge, but found that it needs the roscore to be launched on the server before the bridge can be used for cockpit. Hence roscore can't be launched from cockpit since it is already running. So it seems not much of use, hence abandoned the idea.

* In class, Deebul suggested to look into ROS capabilities. The team decided to look at it since we still haven't managed to run roscore using cockpit.

* In team discussion, we had a difference of opinion on the approach. Lokesh was in favour of finding how to link the html page with ros while I believe if we know how the terminal package in cockpit works then it can be utilized. Lokesh suggested that this approach is tedious since requires extensive knowledge of cockpit, which I feel is true. Finally decided that this week, we will work on our individual approaches and compare our results next week for choosing the best.

* Also have the assignement from Deebul to generate pull requests and reviewing them on git.

* I personally will try on both Lokesh's as well as my original idea to get a breakthrough.

* Reviewed pull request from lokesh and merged it to master.

* Generated a pull request for adding name into READme file.

* Learning about web APIs. This link is helpuful https://developer.mozilla.org/en-US/docs/Web/API

-------------------------------------------------------------------------------------------------
