export PATH := /usr/local/bin:$(PATH)
XBUILD=xbuild
CONFIG=Release

KSPDIR=/home/danapple/KSP/MyMOD/
INSTALLDIR=/home/danapple/KSP/KSP_linux/GameData/KSPSerialIO
CONFIGDIR=$(INSTALLDIR)/PluginData/KSPSerialIO

PLUGINVERSION=$(shell egrep "^\[.*AssemblyVersion" KSPSerialIO/Properties/AssemblyInfo.cs|cut -d\" -f2)
PACKAGEDIR=package/KSPSerialIO
PACKAGECONFIGDIR=$(PACKAGEDIR)/PluginData/KSPSerialIO

all: KSPSerial.dll

KSPSerial.dll:
	$(XBUILD) /p:Configuration=$(CONFIG) /p:TargetFrameworkVersion="v4.5"

install:
	mkdir -p $(INSTALLDIR)
	cp KSPSerialIO/bin/$(CONFIG)/KSPSerialIO.dll $(INSTALLDIR)
	cp KSPSerialIO/bin/$(CONFIG)/PsimaxSerial.dll $(INSTALLDIR)
	#cp ../PsiMaxSerial/PsiMaxSerial/Release/Mono.Posix.dll $(INSTALLDIR)
	mkdir -p $(CONFIGDIR)
	cp config.xml $(CONFIGDIR)

clean:
	$(XBUILD) /p:Configuration=$(CONFIG) /t:Clean

package: all
	mkdir -p $(PACKAGECONFIGDIR)
	cp KSPSerialIO/bin/$(CONFIG)/KSPSerialIO.dll $(PACKAGEDIR)
	cp KSPSerialIO/bin/$(CONFIG)/PsimaxSerial.dll $(PACKAGEDIR)
	#cp ../PsimaxSerial/PsimaxSerial/bin/Release/Mono.Posix.dll $(PACKAGEDIR)
	cp config.xml $(PACKAGECONFIGDIR)
	cd package; zip -r -9 ../KSPSerialIO-cross-$(PLUGINVERSION).zip KSPSerialIO
	rm -r package
	echo $(PLUGINVERSION) > KSPSerialIO.version

