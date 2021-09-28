
README.md 

#---------------------------------------------------------------------------------------------------------------------------
In this commit, you should use your README.md to clearly communicate the following information to your grader:
1. The names and @ucsc.edu emails of every member of the group.

2. A crystal-clear explanation of how to run this. Eg: "Run ’python quadcopterAlphaDemo.py’"

3. A clear explanation of what happens when we run this. If plots show up, what do the plotsrepresent? If we’re running the GUI, what should I do in the GUI to test your work? What should I expect to see?

4. An overview of the state of completion of your project.  What parts of your project work? What parts are coded but buggy? What do you still have left to do?
#---------------------------------------------------------------------------------------------------------------------------

1.) 
Matthew Bennett mabennet@ucsc.edu, 
Brandon Mee-Lee bmeelee@ucsc.edu,
Chris Massiello cmassiel@ucsc.edu,
Rembert Sison rpsison@ucsc.edu


#---------------------------------------------------------------------------------------------------------------------------

2.) To run this system. Simply run the file MVP Test.py

We recommend running this file by navigating to the folder BennettCodeBase in your preferred terminal.
I have been using git bash to test using the line:
	PY 'MVP Test.py'
Entered into the terminal.

When ran the code will prompt you to 
	"Enter a target Pn value: "
When this prompt opens type an integer value between 100 and 5000 and hit enter.

It will then prompt:
	"Enter a target Pe value:"
Once again type an integer value between 100 and 5000 and hit enter.

	If you enter an invalid value IE a too large or too small of value to the system the system will respond with a warning that says
			"Invalid target! Target is TOO CLOSE.
			Please input a Pn that is > 100 or < -100,  and > 100 or < -100"
			
		or
			"Invalid target! Target is TOO FAR.
			Please input a Pn < 5,000 or > -5,000 and < 5,000 or > -5,000"
			
		as appropriate.
		It will then reprompt you to "Enter a target Pn value: "
		
*** IMPORTANT NOTE ******************************************************************************************************

DO NOT input any spaces or other text characters when inputting your integer value. This will cause the system to crash.

*** IMPORTANT NOTE ******************************************************************************************************



#---------------------------------------------------------------------------------------------------------------------------


3.) When ran 4 figures will be opened in ascneding order with figure 4 opening on top.
	
	Figure 4 will be a 3D plot of the position of the UAV and the payload throughout a simulated delivery process. 
	This will show that the controller is able to: 
		Get a target location
		Calculate a drop point destination for the UAV to navigate to
		Control the UAV to fly towards that drop point
		Detect when the UAV is at the drop point and drop the package
		
		Finally this will show how accuarate the drop is.
		In this test case we set the target to be at position Pn = 500 and Pe = 500.
		In testing we have been able to drop the payload within 1 meter of the target location.
		
		The following 3 figures are:
			Figure 3: The commanded and actual course angle (chi) of the UAV.
				This shows the plane navigating on a course to its drop point
				After releasing the package the plane will resume flying at a course angle of 0.
			Figure 2:
				This shows a 2d plot of the Pe of the Payload and of the UAV we can see in this graph:
				Before the drop the payload will match the position of the UAV.
				The payload will drop and begin to diverrge from the UAV and remain at its dropped PE location once it hits the ground. 
				The planes PE in this graph aligns along roughly the same position of the payload since the UAV will follow a drop by flying straight North.
			Figure 1:
				This shows a 2d plot of the Pn of the Payload and the UAV.
				Before the drop the payload will match the postion of the UAV.
				After the drop the payload will diverge from the plane and eventually stop at its final position once it hits the ground.


#---------------------------------------------------------------------------------------------------------------------------

4.) The project is currently at a completed state for only a simplified model of payload flight with no drag forces acting on the payload.
	To put a system like this into practical use we would expect reduced accuracy as this model does not model any affects of drag as this model essentially represents the package falling through a vacuum.
	Thus this system would only be able to be use on a package with exceptional aerodynamics and or one with significant mass that would allow it to not be significantly affected by drag.
	Moving forward we hope to add new systems to the project such as:
		We want to model aerodynamic affects on the payload to see how the effects of air will impact the accuracy of our payload using trhe current drop point calculation.
		Following that we hope to correct the drop point calculation to account for the air resistance of the package in no wind.
		Following this we hope to model the effects of a constant wind would have on flight and correct for those effects with our drop point calculation
		Following this we hope to work with a different aerodynamic model, one with a parachute that would drastically reduce falling speed to make this model work for more delicate packages.
		Following this we hope to allow the system to deliver to a moving target, hypothetically to a moving truck.
		Following that we hope to model this in the simulator to visualize the system as it flies and add "personal flair" to the demo

