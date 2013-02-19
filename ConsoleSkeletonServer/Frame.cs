/**
 * Autogenerated by Thrift Compiler (0.9.0)
 *
 * DO NOT EDIT UNLESS YOU ARE SURE THAT YOU KNOW WHAT YOU ARE DOING
 *  @generated
 */
using System;
using System.Collections;
using System.Collections.Generic;
using System.Text;
using System.IO;
using Thrift;
using Thrift.Collections;
using System.Runtime.Serialization;
using Thrift.Protocol;
using Thrift.Transport;

namespace Jp.Digitalmuseum.Kinect
{

  #if !SILVERLIGHT
  [Serializable]
  #endif
  public partial class Frame : TBase
  {
    private int _frameId;
    private List<byte> _image;
    private List<Joint> _joints;
    private THashSet<string> _keywords;

    public int FrameId
    {
      get
      {
        return _frameId;
      }
      set
      {
        __isset.frameId = true;
        this._frameId = value;
      }
    }

    public List<byte> Image
    {
      get
      {
        return _image;
      }
      set
      {
        __isset.image = true;
        this._image = value;
      }
    }

    public List<Joint> Joints
    {
      get
      {
        return _joints;
      }
      set
      {
        __isset.joints = true;
        this._joints = value;
      }
    }

    public THashSet<string> Keywords
    {
      get
      {
        return _keywords;
      }
      set
      {
        __isset.keywords = true;
        this._keywords = value;
      }
    }


    public Isset __isset;
    #if !SILVERLIGHT
    [Serializable]
    #endif
    public struct Isset {
      public bool frameId;
      public bool image;
      public bool joints;
      public bool keywords;
    }

    public Frame() {
    }

    public void Read (TProtocol iprot)
    {
      TField field;
      iprot.ReadStructBegin();
      while (true)
      {
        field = iprot.ReadFieldBegin();
        if (field.Type == TType.Stop) { 
          break;
        }
        switch (field.ID)
        {
          case 1:
            if (field.Type == TType.I32) {
              FrameId = iprot.ReadI32();
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 2:
            if (field.Type == TType.List) {
              {
                Image = new List<byte>();
                TList _list0 = iprot.ReadListBegin();
                for( int _i1 = 0; _i1 < _list0.Count; ++_i1)
                {
                  byte _elem2 = 0;
                  _elem2 = iprot.ReadByte();
                  Image.Add(_elem2);
                }
                iprot.ReadListEnd();
              }
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.List) {
              {
                Joints = new List<Joint>();
                TList _list3 = iprot.ReadListBegin();
                for( int _i4 = 0; _i4 < _list3.Count; ++_i4)
                {
                  Joint _elem5 = new Joint();
                  _elem5 = new Joint();
                  _elem5.Read(iprot);
                  Joints.Add(_elem5);
                }
                iprot.ReadListEnd();
              }
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 4:
            if (field.Type == TType.Set) {
              {
                Keywords = new THashSet<string>();
                TSet _set6 = iprot.ReadSetBegin();
                for( int _i7 = 0; _i7 < _set6.Count; ++_i7)
                {
                  string _elem8 = null;
                  _elem8 = iprot.ReadString();
                  Keywords.Add(_elem8);
                }
                iprot.ReadSetEnd();
              }
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          default: 
            TProtocolUtil.Skip(iprot, field.Type);
            break;
        }
        iprot.ReadFieldEnd();
      }
      iprot.ReadStructEnd();
    }

    public void Write(TProtocol oprot) {
      TStruct struc = new TStruct("Frame");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      if (__isset.frameId) {
        field.Name = "frameId";
        field.Type = TType.I32;
        field.ID = 1;
        oprot.WriteFieldBegin(field);
        oprot.WriteI32(FrameId);
        oprot.WriteFieldEnd();
      }
      if (Image != null && __isset.image) {
        field.Name = "image";
        field.Type = TType.List;
        field.ID = 2;
        oprot.WriteFieldBegin(field);
        {
          oprot.WriteListBegin(new TList(TType.Byte, Image.Count));
          foreach (byte _iter9 in Image)
          {
            oprot.WriteByte(_iter9);
          }
          oprot.WriteListEnd();
        }
        oprot.WriteFieldEnd();
      }
      if (Joints != null && __isset.joints) {
        field.Name = "joints";
        field.Type = TType.List;
        field.ID = 3;
        oprot.WriteFieldBegin(field);
        {
          oprot.WriteListBegin(new TList(TType.Struct, Joints.Count));
          foreach (Joint _iter10 in Joints)
          {
            _iter10.Write(oprot);
          }
          oprot.WriteListEnd();
        }
        oprot.WriteFieldEnd();
      }
      if (Keywords != null && __isset.keywords) {
        field.Name = "keywords";
        field.Type = TType.Set;
        field.ID = 4;
        oprot.WriteFieldBegin(field);
        {
          oprot.WriteSetBegin(new TSet(TType.String, Keywords.Count));
          foreach (string _iter11 in Keywords)
          {
            oprot.WriteString(_iter11);
          }
          oprot.WriteSetEnd();
        }
        oprot.WriteFieldEnd();
      }
      oprot.WriteFieldStop();
      oprot.WriteStructEnd();
    }

    public override string ToString() {
      StringBuilder sb = new StringBuilder("Frame(");
      sb.Append("FrameId: ");
      sb.Append(FrameId);
      sb.Append(",Image: ");
      sb.Append(Image);
      sb.Append(",Joints: ");
      sb.Append(Joints);
      sb.Append(",Keywords: ");
      sb.Append(Keywords);
      sb.Append(")");
      return sb.ToString();
    }

  }

}
