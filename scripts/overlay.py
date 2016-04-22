from subprocess import call

# helper for applying device tree binaries/overlays to beaglebone cape manager
class Overlay(object):
    slots = '/sys/devices/bone_capemgr.9/slots'
    @staticmethod
    def apply(overlay):
        if ',' + overlay + '\n' in open(Overlay.slots).read():
            return False
        call("echo " + overlay + " > " + Overlay.slots, shell=True)
        return True
