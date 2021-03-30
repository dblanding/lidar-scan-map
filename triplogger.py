import datetime
import shutil

class TripLog():
    """Write a trip log file in .md format.

    Create a new folder (w/ timestamp name) in the current directory.
    Save the completed logfile in the new folder.
    Copy the Maps folder into the new folder."""

    def __init__(self):
        now = datetime.datetime.now()
        date_string = now.strftime("-%y-%m-%d-%H-%M")
        self.folder = "triplog" + date_string
        self.mapfolder = self.folder + "/Maps"
        self.logfilename = self.folder + "/trip_log.md"
        self.text = []  # list of text lines

    def addline(self, line=''):
        """Add line to self.text"""
        self.text.append(line + '  \n')

    def addplot(self, nn):
        """Add a link to plot image (sequence nn)"""
        # Single digit integers need a leading zero
        if nn < 10:
            nn = '0' + str(nn)
        else:
            nn = str(nn)
        name = "Map Plot " + nn
        path = "Maps/scanMap" + nn + ".png"
        line = f"![{name}]({path})  \n"
        self.text.append(line)

    def write(self):
        """Create containing folder with timestamp name
        Copy maps into maps subfolder.
        Write all lines in self.text to logfilename."""
        shutil.copytree("Maps", self.mapfolder)
        with open(self.logfilename, 'w') as file:
            file.writelines(self.text)

