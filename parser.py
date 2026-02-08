def openRawTextFile():
    with open("ghacks/Console_2026-02-07_13-44-13.txt", "r", errors="replace") as file:
        filestring = file.read()
        splitfilelist = filestring.split("#")
        del splitfilelist[0]
        newlist = []
        for text in splitfilelist:
            newlist.append(text[:208])
        return newlist


def convertListToCSV(newlist):
    with open("ghacks/parsed_data.csv", "w") as csv:
        for element in newlist:
            csv.write(element + "\n")


if __name__ == "__main__":
    mylist = openRawTextFile()
    convertListToCSV(mylist)