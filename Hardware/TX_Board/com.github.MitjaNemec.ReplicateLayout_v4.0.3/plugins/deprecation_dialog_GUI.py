# -*- coding: utf-8 -*-

###########################################################################
## Python code generated with wxFormBuilder (version 4.2.1-0-g80c4cb6)
## http://www.wxformbuilder.org/
##
## PLEASE DO *NOT* EDIT THIS FILE!
###########################################################################

import wx
import wx.xrc

###########################################################################
## Class DeprecationDialogGUI
###########################################################################

class DeprecationDialogGUI ( wx.Dialog ):

    def __init__( self, parent ):
        wx.Dialog.__init__ ( self, parent, id = wx.ID_ANY, title = u"Replicate Layout", pos = wx.DefaultPosition, size = wx.Size( 458,212 ), style = wx.DEFAULT_DIALOG_STYLE )

        self.SetSizeHints( wx.DefaultSize, wx.DefaultSize )

        bSizer1 = wx.BoxSizer( wx.VERTICAL )

        self.m_staticText1 = wx.StaticText( self, wx.ID_ANY, u"<b>Deprecation warrning</b>", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText1.SetLabelMarkup( u"<b>Deprecation warrning</b>" )
        self.m_staticText1.Wrap( -1 )

        bSizer1.Add( self.m_staticText1, 0, wx.ALL, 5 )

        self.m_staticText3 = wx.StaticText( self, wx.ID_ANY, u"This plugin will only be supported until KiCad 10.0 is released", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText3.Wrap( -1 )

        bSizer1.Add( self.m_staticText3, 0, wx.ALL, 5 )

        self.m_staticText4 = wx.StaticText( self, wx.ID_ANY, u"After KiCad 10.0 is released I will neither port the plugin neither continue with maintenance.", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText4.Wrap( -1 )

        self.m_staticText4.SetMinSize( wx.Size( -1,40 ) )

        bSizer1.Add( self.m_staticText4, 0, wx.ALL, 5 )


        bSizer1.Add( ( 0, 0), 1, wx.EXPAND, 5 )

        self.m_staticText41 = wx.StaticText( self, wx.ID_ANY, u"Mitja Nemec", wx.DefaultPosition, wx.DefaultSize, 0 )
        self.m_staticText41.Wrap( -1 )

        bSizer1.Add( self.m_staticText41, 0, wx.ALL, 5 )

        bSizer3 = wx.BoxSizer( wx.HORIZONTAL )


        bSizer3.Add( ( 0, 0), 1, wx.EXPAND, 5 )

        self.btn_ok = wx.Button( self, wx.ID_OK, u"Ok", wx.DefaultPosition, wx.DefaultSize, 0 )
        bSizer3.Add( self.btn_ok, 0, wx.ALL, 5 )


        bSizer3.Add( ( 0, 0), 1, wx.EXPAND, 5 )


        bSizer1.Add( bSizer3, 1, wx.EXPAND, 5 )


        self.SetSizer( bSizer1 )
        self.Layout()

        self.Centre( wx.BOTH )

    def __del__( self ):
        pass


