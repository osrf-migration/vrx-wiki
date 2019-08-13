# VRX 2019 Finals


For the VRX Finals, teams will submit their system so that it can be run against previously-unseen scenarios.
Teams have already been exposed to all of the tasks that will be present in the Finals scenarios.
A single system must be designed to solve any of the VRX 2019 tasks.

## What to expect

[The VRX task description](https://bitbucket.org/osrf/vrx/wiki/documentation) document has all the details about the tasks and about the environmental envelope expected in the unseen competition scenarios.

## Preparing your system

To ensure that your system can adapt to previously-unseen scenarios, teams should test their system at least against all released sample tasks.

## Submission process

Each team's system will be evaluated automatically against 18 scenarios.
The submission process is outlined on [the automated evaluation page](https://bitbucket.org/osrf/vrx-docker/src/default/).
Teams must submit their submission files in order to be considered as a participant in the event.
This is the case even if there were no changes since a team's previous submission to a different event.

Submissions will be made though the vrx-events repository. All registered teams must:

### Fork the vrx-events repository

1. Click on [this link](https://bitbucket.org/osrf/vrx-events/fork)
1. You can choose a custom name for the repository, but here we will leave the default value `vrx-events`.
1. After you finish the fork process, you should have a copy of `vrx-events` on https://bitbucket.org/<yourname>/vrx-events.

**Note:** Throughout these tutorials, substitute <yourname> with your Bitbucket account username.

### Clone the vrx-events repository

Great, now you have a copy of the VRX events repository, but it's not very convenient to interact with it through the browser. You want to have it in your computer. You will use the mercurial command line tool to pull that code from the internet to your computer as follows:

* Make sure you have mercurial (hg) installed:

```
sudo apt update
sudo apt install mercurial
```

* Now we use mercurial to "clone" our fork. What the clone command does is copy all the code across all branches from the internet to your computer:

```
hg clone https://bitbucket.org/<yourname>/vrx-docker
```

* Now you should have a local copy of `vrx-events` under `~/vrx-events`. Let's move to that folder:

```
cd ~/vrx-events
```

### Add your submission files

* Navigate to the event where you plan to participate (e.g.: `2019/rehearsal`)

```
cd <year>/<event>
```

**Note:** Substitute <year> and <event> with the year and event that you plan to participate.

* Create a directory with your team name under the current event:

```
mkdir <teamname>
```

**Note:** Substitute <teamname> with your team name.

* Copy all files required for the evaluation.


### Submit your pull request

* Open a pull request for vrx-events on this link:

https://bitbucket.org/osrf/vrx-events/pull-requests/new

Be sure to include the year, name of the event and your team name in the title. E.g.: 2019/rehearsal/team_osrf