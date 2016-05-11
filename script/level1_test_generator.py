REPO_PATH = "C:/AAMyWorkFile/SDK/mcu-sdk-2.0-push"
IP_NAME = "sdhc"


class DriverAPI(object):

    def __init__(self, prototype):
        super(DriverAPI, self).__init__()
        self.retType = ""
        self.ipName = ""
        self.funcName = ""
        self.params = ""

    def getRetType(self):
        return self.retType

    def isTestable(self):
        pass


class Level1TestAPI(object):

    def __init__(self, driverAPI):
        super(Level1TestAPI, self).__init__()
        self.prototype = ""


class Level1TestGenerator(object):

    def __init__(self, repoPath, ipName):
        super(Level1TestGenerator, self).__init__()
        self.repoPath = repoPath
        self.ipName = ipName

    def getDriverAPIs(self):
        filePath = (self.repoPath + "/platform/drivers/" +
                    self.ipName + "/fsl_" + self.ipName + ".h")
        print(filePath)
        try:
            fileHandle = open(filePath, "r")
            fileHandle.close()
        except IOError:
            print("Open file error!")
        else:
            pass
        finally:
            pass

    def getTestAPIs():
        pass

    def outputTestFile():
        pass

    def addCommonTestProject():
        pass

    def addBoardTestProject():
        pass

    def createBoardTestProject():
        pass

print("hello")
testGenerator = Level1TestGenerator(REPO_PATH, IP_NAME)
testGenerator.getDriverAPIs()
