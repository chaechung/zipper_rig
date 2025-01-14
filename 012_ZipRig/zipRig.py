import json
import pymel.core as pm

sel = pm.ls(sl=1)

ASSET_DIR_PATH = r"D:\Dropbox\PythonScripts\012_ZipRig\jason_data"
JSON_FILE_NAME = "ctrl_shape_data.json"

json_file_path = "{0}/{1}".format(ASSET_DIR_PATH, JSON_FILE_NAME)

class DefLib(object):
    @classmethod
    def delShp(cls, SEL):
        for a in SEL:
            aShp = pm.listRelatives(a, s=1)[0]
            pm.parent(aShp, s=1, rm=1)

    def addShp(cls, SEL):
        for a in SEL[1:]:
            aShp = pm.listRelatives(sel[0], s=1)[0]
            pm.parent(aShp, a, s=1, add=1)
    @classmethod
    def duplicateCurve(cls, SEL):
        dupList = []
        for a in SEL:
            dup = pm.duplicate(a, n = a + '_dup')[0]
            dupList.append(dup)
        return dupList
    @classmethod
    def sizeControl(cls, Ctrl, Value):
        pm.hilite(Ctrl)
        ctrlShp = pm.listRelatives(Ctrl, s=1)[0]
        ctrl = pm.select('{}.cv[:]'.format(ctrlShp))
        pm.scale(ctrl, Value[0], Value[1], Value[2], r=1, ocp=1)
        pm.hilite(Ctrl, u=1)
        pm.select(cl=1)
    @classmethod
    def rotateControl(cls, Ctrl, Value):
        pm.hilite(Ctrl)
        ctrlShp = pm.listRelatives(Ctrl, s=1)[0]
        ctrl = pm.select('{}.cv[:]'.format(ctrlShp))
        pm.rotate(ctrl, Value[0], Value[1], Value[2], r=1, ocp=1)
        pm.hilite(Ctrl, u=1)
        pm.select(cl=1)
    @classmethod
    def parentConst(cls, ParentList, ChildrenList):
        for a in range(len(ParentList.keys())):
            const = pm.parentConstraint(ParentList[a], ChildrenList[a], mo=0, w=0)
            const.interpType.set(2)

    @classmethod
    def rebuildCurves(cls, CURVES, CTL_AMNT, DEGREE):
        dupCurves = DefLib.duplicateCurve(CURVES)
        for crv in dupCurves:
            pm.rebuildCurve(crv, ch = 0, rpo=1, rt=0, end= 1, kr=0, kcp=0, kep=1, kt=0 , s= CTL_AMNT, d=DEGREE, tol= 0.01)
        return dupCurves

    @classmethod
    def createLoftCurves(cls, CURVE):
        curve_01 = DefLib.duplicateCurve([CURVE])[0]
        curve_02 = DefLib.duplicateCurve([CURVE])[0]
        curve_01.tx.set(-1)
        curve_02.tx.set(1)

        nrbs = cls.loft([curve_01, curve_02])
        pm.delete(curve_01, curve_02)
        return nrbs
    @classmethod
    def loft(cls, CURVES):
        nrbs = pm.loft(CURVES, ch=0, u=1, c=0, ar=1, d=3, ss=1, rn=0, po=0, rsn=1)
        return nrbs
    @classmethod
    def createFollicles(cls, NRBS, NUM):
        '''
        Args:
            NRBS: Nurbs Surface
            NUM: Number of follicles

        Returns: follicles on Nurbs Surface

        '''
        FolDic = {}
        for a in range(NUM):
            ParamU= (1.0000/(NUM-1))*a
            FolShape = pm.createNode("follicle", n='fol_0{}_flcShape'.format(str(a+1)))
            Follicle= pm.listRelatives(FolShape, p=True)[0]
            NurbsShape= pm.listRelatives(NRBS, s=1)[0]
            pm.setAttr(Follicle + ".visibility", 0)
            pm.setAttr(FolShape+".parameterU", ParamU)
            pm.setAttr(FolShape + ".parameterV", 0.5)
            FolShape.outRotate >> Follicle.rotate
            FolShape.outTranslate >> Follicle.translate
            NurbsShape.worldMatrix[0] >> FolShape.inputWorldMatrix
            NurbsShape.local >> FolShape.inputSurface
            FolDic[a] = Follicle
        return FolDic
    @classmethod
    def createJoints(cls, LIST):
        jntDic = {}
        for a in range(len(LIST)):
            pos = pm.xform(LIST[a], m=1, q=1, ws=1)
            pm.select(cl=1)
            jnt = pm.joint()
            pm.xform(jnt, m=pos, ws=1)
            jntDic[a] = jnt
        return jntDic
    @classmethod
    def getCVPosition(cls, CURVE):
        spans = pm.getAttr('{}.spans'.format(CURVE))
        cv_dic = {}
        for cv in range(spans + 1):
            cv_pos = pm.pointPosition('{}.cv[{}]'.format(CURVE, cv), w=1)
            cv_dic[cv] = cv_pos
        return cv_dic
    @classmethod
    def createControl(cls, POSITION, Type, Color):
        ctl = CrvLib.createCurveCtl(Type, Color)
        jnt = pm.joint()
        off = pm.group(em=1)

        pm.parent(ctl, off)
        pm.xform(off, t=POSITION, ws=1)
        dic = {'ctl': ctl, 'off': off, 'jnt': jnt}
        return dic
    @classmethod
    def grouping(cls, CONTENTS, NAME):
        pm.select(CONTENTS)
        if pm.objExists(NAME):
            grp = NAME
            pm.parent(CONTENTS, NAME)
        else:
            grp = pm.group(n=NAME)
        return grp
    @classmethod
    def remapValue(cls, SEL, NUM, CTL):
        dist = 1.0000/NUM
        for a in range(len(SEL)):
            aConst = pm.listConnections(SEL[a], type='constraint')[0]
            constTargets = pm.parentConstraint(aConst, q=1, targetList=1)
            close = '{}W0'.format(constTargets[0])
            open = '{}W1'.format(constTargets[1])

            rmpV = pm.createNode('remapValue', n=SEL[a].replace('jnt', 'RMVP'))
            rvrs = pm.createNode('reverse', n=SEL[a].replace('jnt', 'RVRS'))

            startV = dist*a
            endV = dist*(a+1)
            rmpV.inputMin.set(startV)
            rmpV.inputMax.set(endV)

            CTL.parameterU >> rmpV.inputValue
            rmpV.outValue >> rvrs.inputX
            rmpV.outValue >> '{}.{}'.format(aConst, open)
            rvrs.outputX >> '{}.{}'.format(aConst, close)

class CrvLib(object):
    """
    Shape :
    'sphereDumbell', 'squareDumbell', 'box'

    Color :
    'brightYellow', 'pink', 'skyBlue', 'red'
    """
    @classmethod
    def load_assets_from_json(cls):
        with open(json_file_path, "r") as file_for_read:
            assets = json.load(file_for_read)
        return assets

    @classmethod
    def createCurveCtl(cls, Type, Color):
        CrvData = cls.load_assets_from_json()['Shape']
        ColorData = cls.load_assets_from_json()['Color']
        CV_value = CrvData[Type]
        Color_value = ColorData[Color]

        crv = pm.curve(d=1, p = CV_value)
        crvShp = pm.listRelatives(crv, s=1)[0]
        crvShp.overrideEnabled.set(1)
        crvShp.overrideColor.set(Color_value)
        return crv

def buildRig(PREFIX, CURVES, CTL_AMNT, PULLER, DIRECTION, DELCRV):
    #--- Direction x -> lower, upper / Direction y -> left, right / Direction z -> front, back
    if DIRECTION == 'x':
        side_01 = 'DW'
        side_02 = 'UP'
    if DIRECTION == 'y':
        side_01 = 'L'
        side_02 = 'R'
    if DIRECTION == 'z':
        side_01 = 'FR'
        side_02 = 'BK'

    dupCurves = DefLib.rebuildCurves(CURVES, (int(CTL_AMNT)-1)*2, 3)

    #--- Creating Nurbs Surfaces from rebuild curves ---#
    nrbs_01 = DefLib.createLoftCurves(dupCurves[0])
    nrbs_02 = DefLib.createLoftCurves(dupCurves[1])
    nrbs_03 = DefLib.createLoftCurves(dupCurves[2])
    pm.delete(dupCurves)

    #--- Creating follicles on the surfaces amount : Spans X2---#
    fol_01 = DefLib.createFollicles(nrbs_01, (int(CTL_AMNT)-1)*3+1)
    DefLib.grouping(fol_01.values(), '{}_close_M_fol_grp'.format(PREFIX))
    fol_02 = DefLib.createFollicles(nrbs_02, (int(CTL_AMNT)-1)*3+1)
    DefLib.grouping(fol_02.values(), '{}_open_{}_fol_grp'.format(PREFIX, side_01))
    fol_03 = DefLib.createFollicles(nrbs_03, (int(CTL_AMNT)-1)*3+1)
    DefLib.grouping(fol_03.values(), '{}_open_{}_fol_grp'.format(PREFIX, side_02))
    #--- Renaming follicles ---#
    for a in range(len(fol_01.keys())):
        pm.rename(fol_01[a], '{}_close_M_0{}_fol'.format(PREFIX, a+1))
    for a in range(len(fol_02.keys())):
        pm.rename(fol_02[a], '{}_open_{}_0{}_fol'.format(PREFIX, side_01, a+1))
    for a in range(len(fol_03.keys())):
        pm.rename(fol_03[a], '{}_open_{}_0{}_fol'.format(PREFIX, side_02, a+1))

    #--- Creating Controls for Nurbs Surfaces ---#
    dupCurves = DefLib.rebuildCurves(CURVES, int(CTL_AMNT)-1, 1)
    DefLib.grouping(dupCurves, '{}_temp_crv_grp'.format(PREFIX))
    for a in range(len(dupCurves)):
        pm.rename(dupCurves[a], '{}_0{}_temp_crv'.format(PREFIX, a+1))
        aShp = pm.listRelatives(dupCurves[a], s=1)[0]
        aShp.template.set(1)
    cv_00_pos = DefLib.getCVPosition(dupCurves[0])
    cv_01_pos = DefLib.getCVPosition(dupCurves[1])
    cv_02_pos = DefLib.getCVPosition(dupCurves[2])
    ctlList_01 = []
    for a in range(len(cv_00_pos)):
        Ctl = DefLib.createControl(cv_00_pos.values()[a], 'squareDumbell', 'brightYellow')
        DefLib.rotateControl(Ctl['ctl'], [90, 0, 0])
        pm.rename(Ctl['ctl'], '{}_M_0{}_ctl'.format(PREFIX, a+1))
        pm.rename(Ctl['jnt'], '{}_M_0{}_ctl_jnt'.format(PREFIX, a+1))
        pm.rename(Ctl['off'], '{}_M_0{}_ctl_off'.format(PREFIX, a+1))
        ctlList_01.append(Ctl)
    ctlList_02 = []
    for a in range(len(cv_01_pos)):
        Ctl = DefLib.createControl(cv_01_pos.values()[a], 'sphereDumbell', 'skyBlue')
        DefLib.rotateControl(Ctl['ctl'], [90, 0, 0])
        pm.rename(Ctl['ctl'], '{}_{}_0{}_ctl'.format(PREFIX, side_01, a+1))
        pm.rename(Ctl['jnt'], '{}_{}_0{}_ctl_jnt'.format(PREFIX, side_01, a + 1))
        pm.rename(Ctl['off'], '{}_{}_0{}_ctl_off'.format(PREFIX, side_01, a+1))
        ctlList_02.append(Ctl)
    ctlList_03 = []
    for a in range(len(cv_02_pos)):
        Ctl = DefLib.createControl(cv_02_pos.values()[a], 'sphereDumbell', 'pink')
        DefLib.rotateControl(Ctl['ctl'], [90, 0, 0])
        pm.rename(Ctl['ctl'], '{}_{}_0{}_ctl'.format(PREFIX, side_02, a+1))
        pm.rename(Ctl['jnt'], '{}_{}_0{}_ctl_jnt'.format(PREFIX, side_02, a + 1))
        pm.rename(Ctl['off'], '{}_{}_0{}_ctl_off'.format(PREFIX, side_02, a+1))
        ctlList_03.append(Ctl)

    for a in range(len(ctlList_01)):
        pm.parent(ctlList_02[a]['off'], ctlList_01[a]['ctl'])
        pm.parent(ctlList_03[a]['off'], ctlList_01[a]['ctl'])

    pm.select([a['off'] for a in ctlList_01])
    pm.group(n = '{}_M_ctl_grp'.format(PREFIX))

    #--- Creating bind skin joints amount : Spans X2---#
    jnt_01 = DefLib.createJoints(fol_01)
    jnt_02 = DefLib.createJoints(fol_01)
    for a in range(len(jnt_01.keys())):
        pm.rename(jnt_01[a], '{}_{}_0{}_jnt'.format(PREFIX, side_01, a+1))
    for a in range(len(jnt_02.keys())):
        pm.rename(jnt_02[a], '{}_{}_0{}_jnt'.format(PREFIX, side_02, a+1))

    pm.select(jnt_01.values())
    pm.group(n='{}_bind_jnt_{}_grp'.format(PREFIX, side_01))
    pm.select(jnt_02.values())
    pm.group(n='{}_bind_jnt_{}_grp'.format(PREFIX, side_02))

    #--- ParentConstraint on bind skin joints with close / open follicles ---#
    DefLib.parentConst(fol_01, jnt_01)
    DefLib.parentConst(fol_02, jnt_01)

    DefLib.parentConst(fol_01, jnt_02)
    DefLib.parentConst(fol_03, jnt_02)

    #--- Create Puller Controls ---#
    #--- Connect Puller Controls to follicle's U or V parameter ---#
    #--- Connect U or V parameter to remapValue nodes & reverse nodes & parentConstraint targets ---#

    if PULLER:
        puller_ctl_01 = CrvLib.createCurveCtl('box', 'red')
        offGrp_01 = pm.group()
        puller_ctl_02 = CrvLib.createCurveCtl('box', 'red')
        offGrp_02 = pm.group()
        if DIRECTION == 'x':
            print 'x direction'
        elif DIRECTION == 'y':
            print 'y direction'
            pos_01 = pm.xform(jnt_01[0], t=1, q=1, ws=1)
            pm.xform(offGrp_01, t=pos_01, ws=1)
            pos_02 = pm.xform(jnt_01.values()[-1], m=1, q=1, ws=1)
            pm.xform(offGrp_01, m=pos_02, ws=1)
        elif DIRECTION == 'z':
            print 'z driection'
    else:
        puller_ctl_01_cnt = CrvLib.createCurveCtl('box', 'red')
        pm.rename(puller_ctl_01_cnt, 'puller_cnt_ctl')
        cnt_offGrp = pm.group(n=puller_ctl_01_cnt + '_off')
        pm.hide(cnt_offGrp)
        puller_ctl_01 = CrvLib.createCurveCtl('box', 'red')
        pm.rename(puller_ctl_01, 'puller_ctl')
        multOffGrp = pm.group(n = puller_ctl_01+'_mult_off')
        offGrp = pm.group(n = puller_ctl_01+'_off')
        mult = pm.createNode('multiplyDivide', n = '{}_MULT'.format(puller_ctl_01))
        mult.input2X.set(-1)
        mult.input2Y.set(-1)
        mult.input2Z.set(-1)
        puller_ctl_01.translate >> mult.input1
        mult.output >> multOffGrp.translate
        puller_ctl_01.translate >> puller_ctl_01_cnt.translate

        grouping([cnt_offGrp, offGrp], '{}_M_ctl_grp'.format(PREFIX))
        rmpValue = pm.createNode('remapValue', n = puller_ctl_01 + '_RMPV')
        if DIRECTION == 'x':
            print 'x direction'
        elif DIRECTION == 'y':
            print 'y direction'
            bindJnt = pm.joint(n='{}_M_bind_jnt'.format(PREFIX))
            bindOff = pm.group(n='{}_M_bind_jnt_off'.format(PREFIX))
            pos = pm.xform(jnt_01[0], t=1, q=1, ws=1)
            pm.xform(offGrp, t=pos, ws=1)
            pm.xform(bindOff, t=pos, ws=1)

            default_fol = DefLib.createFollicles(nrbs_01, 2)
            DefLib.grouping(default_fol.values(), '{}_default_fol_grp'.format(PREFIX))

            pm.parentConstraint(default_fol[0], bindOff, mo=1)
            pm.parentConstraint(bindJnt, offGrp, mo=1)

            folShp = pm.listRelatives(default_fol[0], s=1)[0]
            tyPos = pm.xform(jnt_01.values()[-1], t=1, q=1, ws=1)
            rmpValue.inputMax.set(tyPos[1])
            puller_ctl_01_cnt.ty >> rmpValue.inputValue
            rmpValue.outValue >> folShp.parameterU

            # default_fol[0].translate >> offGrp.translate
            # default_fol[0].rotate >> offGrp.rotate

        elif DIRECTION == 'z':
            print 'z driection'
        DefLib.remapValue(jnt_01, (int(CTL_AMNT)-1)*3+1, folShp)
        DefLib.remapValue(jnt_02, (int(CTL_AMNT)-1)*3+1, folShp)

    #--- Bind skin to rebuild curves & Nurbs Surfaces ---#
    crvJnt_01 = [a['jnt'] for a in ctlList_01]
    crvJnt_02 = [a['jnt'] for a in ctlList_02]
    crvJnt_03 = [a['jnt'] for a in ctlList_03]

    pm.skinCluster(crvJnt_01, dupCurves[0],  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 1)[0]
    pm.skinCluster(crvJnt_02, dupCurves[1],  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 1)[0]
    pm.skinCluster(crvJnt_03, dupCurves[2],  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 1)[0]

    pm.skinCluster(crvJnt_01, nrbs_01,  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 3)[0]
    pm.skinCluster(crvJnt_02, nrbs_02,  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 3)[0]
    pm.skinCluster(crvJnt_03, nrbs_03,  toSelectedBones=True, bindMethod=0, skinMethod=0, normalizeWeights=1, mi = 3)[0]

    #--- Grouping and Sorting ---#

    if DELCRV:
        pm.delete(CURVES)
    else:
        pass

    folGrpList = pm.ls('*_fol_grp', type='transform')
    folGrp = DefLib.grouping(folGrpList, '{}_fol_grp'.format(PREFIX))
    nrbsGrpList = [nrbs_01, nrbs_02, nrbs_03]
    nrbsGrp = DefLib.grouping(nrbsGrpList, '{}_nrbs_grp'.format(PREFIX))
    nrbsGrp.visibility.set(0)
    jntGrpList = pm.ls('*jnt_*grp')
    jntGrp = DefLib.grouping([jntGrpList, '{}_M_bind_jnt_off'.format(PREFIX)], '{}_jnt_grp'.format(PREFIX))
    jntGrp.visibility.set(0)
    ctlGrp = pm.ls('{}_M_ctl_grp'.format(PREFIX))
    tempCrvGrp = pm.ls('{}_temp_crv_grp'.format(PREFIX))

    DefLib.grouping([folGrp, nrbsGrp, jntGrp, ctlGrp, tempCrvGrp], '{}_RIG_GRP'.format(PREFIX))