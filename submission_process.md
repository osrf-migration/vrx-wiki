# Submission process

Each team's system will be evaluated automatically against 18 scenarios.
The submission process is outlined on the [automated evaluation page](https://bitbucket.org/osrf/vrx-docker/src/default/).
Teams must submit their submission files in order to be considered as a participant in the event.
This is the case even if there were no changes since a team's previous submission to a different event.

Submissions will be made though the [vrx-events](https://bitbucket.org/osrf/vrx-events) repository. All registered teams must:

### Fork the `vrx-events` repository

1 . Click on [this link](https://bitbucket.org/osrf/vrx-events/fork).

2 . You can choose a custom name for the repository, but here we will leave the default value `vrx-events`.

3 . After you finish the fork process, you should have a copy of `vrx-events` on https://bitbucket.org/<yourname>/vrx-events.

**Note:** Throughout these tutorials, substitute <yourname> with your Bitbucket account username.

### Clone the `vrx-events` repository

Great, now you have a copy of the VRX events repository, but it's not very convenient to interact with it through the browser. You want to have it in your computer. You will use the `mercurial` command line tool to pull that code from the internet to your computer as follows:

1 . Make sure you have `mercurial` (`hg`) installed:

```
sudo apt update
sudo apt install mercurial
```

2 . Now we use mercurial to "clone" our fork. What the clone command does is copy all the code across all branches from the internet to your computer:

```
hg clone https://bitbucket.org/<yourname>/vrx-events
```

3 . Now you should have a local copy of `vrx-events` under `~/vrx-events`. Let's move to that folder:

```
cd ~/vrx-events
```

### Add your submission files

1 . Navigate to the event where you plan to participate (e.g.: `2019/rehearsal`)

```
cd <year>/<event>
```

**Note:** Substitute <year> and <event> with the year and event that you plan to participate.

2 . Create a directory with your team name under the current event:

```
mkdir <teamname>
cd <teamname>
```

**Note:** Substitute <teamname> with your team name.

3 . Copy all files required for the evaluation. Follow the instructions detailed in the event page for the complete list of files to be submitted.


### Submit your pull request

1 . In preparation for your pull request, create a branch, e.g.:

```
hg branch 2019_rehearsal_team_osrf
```

2 . Add your files on the next commit, e.g.:

```
hg add *
```

3 . Verify the changed files:

```
hg status
```

4 . Commit and push your changes to your fork:

hg ci -m"Team OSRF submission for 2019/rehearsal event." -u"Carlos Aguero"
hg push --new

5 . Open a pull request for `vrx-events` on this link:

https://bitbucket.org/<teamname>/vrx-events/pull-requests/new

**Note:** Substitute <teamname> with your team name.

Make sure to select your branch on the left box and target your pull request against the `osrf/vrx-events` repository to the `default` branch.


Be sure to include the year, name of the event and your team name in the title. E.g.: 2019/rehearsal/team_osrf

Click on the `Create pull request` button and wait for your pull request to be approved and merged.

![vrx-events_pr_example.png](https://bitbucket.org/repo/BgXLzgM/images/297094918-vrx-events_pr_example.png)