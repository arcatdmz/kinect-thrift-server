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
  public partial class Joint : TBase
  {
    private JointType _type;
    private Position3D _position;
    private Position2D _screenPosition;

    /// <summary>
    /// 
    /// <seealso cref="JointType"/>
    /// </summary>
    public JointType Type
    {
      get
      {
        return _type;
      }
      set
      {
        __isset.type = true;
        this._type = value;
      }
    }

    public Position3D Position
    {
      get
      {
        return _position;
      }
      set
      {
        __isset.position = true;
        this._position = value;
      }
    }

    public Position2D ScreenPosition
    {
      get
      {
        return _screenPosition;
      }
      set
      {
        __isset.screenPosition = true;
        this._screenPosition = value;
      }
    }


    public Isset __isset;
    #if !SILVERLIGHT
    [Serializable]
    #endif
    public struct Isset {
      public bool type;
      public bool position;
      public bool screenPosition;
    }

    public Joint() {
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
              Type = (JointType)iprot.ReadI32();
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 2:
            if (field.Type == TType.Struct) {
              Position = new Position3D();
              Position.Read(iprot);
            } else { 
              TProtocolUtil.Skip(iprot, field.Type);
            }
            break;
          case 3:
            if (field.Type == TType.Struct) {
              ScreenPosition = new Position2D();
              ScreenPosition.Read(iprot);
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
      TStruct struc = new TStruct("Joint");
      oprot.WriteStructBegin(struc);
      TField field = new TField();
      if (__isset.type) {
        field.Name = "type";
        field.Type = TType.I32;
        field.ID = 1;
        oprot.WriteFieldBegin(field);
        oprot.WriteI32((int)Type);
        oprot.WriteFieldEnd();
      }
      if (Position != null && __isset.position) {
        field.Name = "position";
        field.Type = TType.Struct;
        field.ID = 2;
        oprot.WriteFieldBegin(field);
        Position.Write(oprot);
        oprot.WriteFieldEnd();
      }
      if (ScreenPosition != null && __isset.screenPosition) {
        field.Name = "screenPosition";
        field.Type = TType.Struct;
        field.ID = 3;
        oprot.WriteFieldBegin(field);
        ScreenPosition.Write(oprot);
        oprot.WriteFieldEnd();
      }
      oprot.WriteFieldStop();
      oprot.WriteStructEnd();
    }

    public override string ToString() {
      StringBuilder sb = new StringBuilder("Joint(");
      sb.Append("Type: ");
      sb.Append(Type);
      sb.Append(",Position: ");
      sb.Append(Position== null ? "<null>" : Position.ToString());
      sb.Append(",ScreenPosition: ");
      sb.Append(ScreenPosition== null ? "<null>" : ScreenPosition.ToString());
      sb.Append(")");
      return sb.ToString();
    }

  }

}
