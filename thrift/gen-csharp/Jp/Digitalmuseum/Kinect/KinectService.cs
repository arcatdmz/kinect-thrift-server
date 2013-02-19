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
  public partial class KinectService {
    public interface Iface {
      void addKeyword(string text);
      #if SILVERLIGHT
      IAsyncResult Begin_addKeyword(AsyncCallback callback, object state, string text);
      void End_addKeyword(IAsyncResult asyncResult);
      #endif
      void removeKeyword(string text);
      #if SILVERLIGHT
      IAsyncResult Begin_removeKeyword(AsyncCallback callback, object state, string text);
      void End_removeKeyword(IAsyncResult asyncResult);
      #endif
      THashSet<string> getKeywords();
      #if SILVERLIGHT
      IAsyncResult Begin_getKeywords(AsyncCallback callback, object state, );
      THashSet<string> End_getKeywords(IAsyncResult asyncResult);
      #endif
      void setAngle(int angle);
      #if SILVERLIGHT
      IAsyncResult Begin_setAngle(AsyncCallback callback, object state, int angle);
      void End_setAngle(IAsyncResult asyncResult);
      #endif
      int getAngle();
      #if SILVERLIGHT
      IAsyncResult Begin_getAngle(AsyncCallback callback, object state, );
      int End_getAngle(IAsyncResult asyncResult);
      #endif
      Frame getFrame();
      #if SILVERLIGHT
      IAsyncResult Begin_getFrame(AsyncCallback callback, object state, );
      Frame End_getFrame(IAsyncResult asyncResult);
      #endif
      void stop();
      #if SILVERLIGHT
      IAsyncResult Begin_stop(AsyncCallback callback, object state, );
      void End_stop(IAsyncResult asyncResult);
      #endif
    }

    public class Client : Iface {
      public Client(TProtocol prot) : this(prot, prot)
      {
      }

      public Client(TProtocol iprot, TProtocol oprot)
      {
        iprot_ = iprot;
        oprot_ = oprot;
      }

      protected TProtocol iprot_;
      protected TProtocol oprot_;
      protected int seqid_;

      public TProtocol InputProtocol
      {
        get { return iprot_; }
      }
      public TProtocol OutputProtocol
      {
        get { return oprot_; }
      }


      
      #if SILVERLIGHT
      public IAsyncResult Begin_addKeyword(AsyncCallback callback, object state, string text)
      {
        return send_addKeyword(callback, state, text);
      }

      public void End_addKeyword(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
      }

      #endif

      public void addKeyword(string text)
      {
        #if !SILVERLIGHT
        send_addKeyword(text);

        #else
        var asyncResult = Begin_addKeyword(null, null, text);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_addKeyword(AsyncCallback callback, object state, string text)
      #else
      public void send_addKeyword(string text)
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("addKeyword", TMessageType.Call, seqid_));
        addKeyword_args args = new addKeyword_args();
        args.Text = text;
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_removeKeyword(AsyncCallback callback, object state, string text)
      {
        return send_removeKeyword(callback, state, text);
      }

      public void End_removeKeyword(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
      }

      #endif

      public void removeKeyword(string text)
      {
        #if !SILVERLIGHT
        send_removeKeyword(text);

        #else
        var asyncResult = Begin_removeKeyword(null, null, text);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_removeKeyword(AsyncCallback callback, object state, string text)
      #else
      public void send_removeKeyword(string text)
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("removeKeyword", TMessageType.Call, seqid_));
        removeKeyword_args args = new removeKeyword_args();
        args.Text = text;
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_getKeywords(AsyncCallback callback, object state, )
      {
        return send_getKeywords(callback, state);
      }

      public THashSet<string> End_getKeywords(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
        return recv_getKeywords();
      }

      #endif

      public THashSet<string> getKeywords()
      {
        #if !SILVERLIGHT
        send_getKeywords();
        return recv_getKeywords();

        #else
        var asyncResult = Begin_getKeywords(null, null, );
        return End_getKeywords(asyncResult);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_getKeywords(AsyncCallback callback, object state, )
      #else
      public void send_getKeywords()
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("getKeywords", TMessageType.Call, seqid_));
        getKeywords_args args = new getKeywords_args();
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      public THashSet<string> recv_getKeywords()
      {
        TMessage msg = iprot_.ReadMessageBegin();
        if (msg.Type == TMessageType.Exception) {
          TApplicationException x = TApplicationException.Read(iprot_);
          iprot_.ReadMessageEnd();
          throw x;
        }
        getKeywords_result result = new getKeywords_result();
        result.Read(iprot_);
        iprot_.ReadMessageEnd();
        if (result.__isset.success) {
          return result.Success;
        }
        throw new TApplicationException(TApplicationException.ExceptionType.MissingResult, "getKeywords failed: unknown result");
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_setAngle(AsyncCallback callback, object state, int angle)
      {
        return send_setAngle(callback, state, angle);
      }

      public void End_setAngle(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
      }

      #endif

      public void setAngle(int angle)
      {
        #if !SILVERLIGHT
        send_setAngle(angle);

        #else
        var asyncResult = Begin_setAngle(null, null, angle);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_setAngle(AsyncCallback callback, object state, int angle)
      #else
      public void send_setAngle(int angle)
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("setAngle", TMessageType.Call, seqid_));
        setAngle_args args = new setAngle_args();
        args.Angle = angle;
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_getAngle(AsyncCallback callback, object state, )
      {
        return send_getAngle(callback, state);
      }

      public int End_getAngle(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
        return recv_getAngle();
      }

      #endif

      public int getAngle()
      {
        #if !SILVERLIGHT
        send_getAngle();
        return recv_getAngle();

        #else
        var asyncResult = Begin_getAngle(null, null, );
        return End_getAngle(asyncResult);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_getAngle(AsyncCallback callback, object state, )
      #else
      public void send_getAngle()
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("getAngle", TMessageType.Call, seqid_));
        getAngle_args args = new getAngle_args();
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      public int recv_getAngle()
      {
        TMessage msg = iprot_.ReadMessageBegin();
        if (msg.Type == TMessageType.Exception) {
          TApplicationException x = TApplicationException.Read(iprot_);
          iprot_.ReadMessageEnd();
          throw x;
        }
        getAngle_result result = new getAngle_result();
        result.Read(iprot_);
        iprot_.ReadMessageEnd();
        if (result.__isset.success) {
          return result.Success;
        }
        throw new TApplicationException(TApplicationException.ExceptionType.MissingResult, "getAngle failed: unknown result");
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_getFrame(AsyncCallback callback, object state, )
      {
        return send_getFrame(callback, state);
      }

      public Frame End_getFrame(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
        return recv_getFrame();
      }

      #endif

      public Frame getFrame()
      {
        #if !SILVERLIGHT
        send_getFrame();
        return recv_getFrame();

        #else
        var asyncResult = Begin_getFrame(null, null, );
        return End_getFrame(asyncResult);

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_getFrame(AsyncCallback callback, object state, )
      #else
      public void send_getFrame()
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("getFrame", TMessageType.Call, seqid_));
        getFrame_args args = new getFrame_args();
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

      public Frame recv_getFrame()
      {
        TMessage msg = iprot_.ReadMessageBegin();
        if (msg.Type == TMessageType.Exception) {
          TApplicationException x = TApplicationException.Read(iprot_);
          iprot_.ReadMessageEnd();
          throw x;
        }
        getFrame_result result = new getFrame_result();
        result.Read(iprot_);
        iprot_.ReadMessageEnd();
        if (result.__isset.success) {
          return result.Success;
        }
        throw new TApplicationException(TApplicationException.ExceptionType.MissingResult, "getFrame failed: unknown result");
      }

      
      #if SILVERLIGHT
      public IAsyncResult Begin_stop(AsyncCallback callback, object state, )
      {
        return send_stop(callback, state);
      }

      public void End_stop(IAsyncResult asyncResult)
      {
        oprot_.Transport.EndFlush(asyncResult);
      }

      #endif

      public void stop()
      {
        #if !SILVERLIGHT
        send_stop();

        #else
        var asyncResult = Begin_stop(null, null, );

        #endif
      }
      #if SILVERLIGHT
      public IAsyncResult send_stop(AsyncCallback callback, object state, )
      #else
      public void send_stop()
      #endif
      {
        oprot_.WriteMessageBegin(new TMessage("stop", TMessageType.Call, seqid_));
        stop_args args = new stop_args();
        args.Write(oprot_);
        oprot_.WriteMessageEnd();
        #if SILVERLIGHT
        return oprot_.Transport.BeginFlush(callback, state);
        #else
        oprot_.Transport.Flush();
        #endif
      }

    }
    public class Processor : TProcessor {
      public Processor(Iface iface)
      {
        iface_ = iface;
        processMap_["addKeyword"] = addKeyword_Process;
        processMap_["removeKeyword"] = removeKeyword_Process;
        processMap_["getKeywords"] = getKeywords_Process;
        processMap_["setAngle"] = setAngle_Process;
        processMap_["getAngle"] = getAngle_Process;
        processMap_["getFrame"] = getFrame_Process;
        processMap_["stop"] = stop_Process;
      }

      protected delegate void ProcessFunction(int seqid, TProtocol iprot, TProtocol oprot);
      private Iface iface_;
      protected Dictionary<string, ProcessFunction> processMap_ = new Dictionary<string, ProcessFunction>();

      public bool Process(TProtocol iprot, TProtocol oprot)
      {
        try
        {
          TMessage msg = iprot.ReadMessageBegin();
          ProcessFunction fn;
          processMap_.TryGetValue(msg.Name, out fn);
          if (fn == null) {
            TProtocolUtil.Skip(iprot, TType.Struct);
            iprot.ReadMessageEnd();
            TApplicationException x = new TApplicationException (TApplicationException.ExceptionType.UnknownMethod, "Invalid method name: '" + msg.Name + "'");
            oprot.WriteMessageBegin(new TMessage(msg.Name, TMessageType.Exception, msg.SeqID));
            x.Write(oprot);
            oprot.WriteMessageEnd();
            oprot.Transport.Flush();
            return true;
          }
          fn(msg.SeqID, iprot, oprot);
        }
        catch (IOException)
        {
          return false;
        }
        return true;
      }

      public void addKeyword_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        addKeyword_args args = new addKeyword_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        iface_.addKeyword(args.Text);
        return;
      }
      public void removeKeyword_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        removeKeyword_args args = new removeKeyword_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        iface_.removeKeyword(args.Text);
        return;
      }
      public void getKeywords_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        getKeywords_args args = new getKeywords_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        getKeywords_result result = new getKeywords_result();
        result.Success = iface_.getKeywords();
        oprot.WriteMessageBegin(new TMessage("getKeywords", TMessageType.Reply, seqid)); 
        result.Write(oprot);
        oprot.WriteMessageEnd();
        oprot.Transport.Flush();
      }

      public void setAngle_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        setAngle_args args = new setAngle_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        iface_.setAngle(args.Angle);
        return;
      }
      public void getAngle_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        getAngle_args args = new getAngle_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        getAngle_result result = new getAngle_result();
        result.Success = iface_.getAngle();
        oprot.WriteMessageBegin(new TMessage("getAngle", TMessageType.Reply, seqid)); 
        result.Write(oprot);
        oprot.WriteMessageEnd();
        oprot.Transport.Flush();
      }

      public void getFrame_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        getFrame_args args = new getFrame_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        getFrame_result result = new getFrame_result();
        result.Success = iface_.getFrame();
        oprot.WriteMessageBegin(new TMessage("getFrame", TMessageType.Reply, seqid)); 
        result.Write(oprot);
        oprot.WriteMessageEnd();
        oprot.Transport.Flush();
      }

      public void stop_Process(int seqid, TProtocol iprot, TProtocol oprot)
      {
        stop_args args = new stop_args();
        args.Read(iprot);
        iprot.ReadMessageEnd();
        iface_.stop();
        return;
      }
    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class addKeyword_args : TBase
    {
      private string _text;

      public string Text
      {
        get
        {
          return _text;
        }
        set
        {
          __isset.text = true;
          this._text = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool text;
      }

      public addKeyword_args() {
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
              if (field.Type == TType.String) {
                Text = iprot.ReadString();
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
        TStruct struc = new TStruct("addKeyword_args");
        oprot.WriteStructBegin(struc);
        TField field = new TField();
        if (Text != null && __isset.text) {
          field.Name = "text";
          field.Type = TType.String;
          field.ID = 1;
          oprot.WriteFieldBegin(field);
          oprot.WriteString(Text);
          oprot.WriteFieldEnd();
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("addKeyword_args(");
        sb.Append("Text: ");
        sb.Append(Text);
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class removeKeyword_args : TBase
    {
      private string _text;

      public string Text
      {
        get
        {
          return _text;
        }
        set
        {
          __isset.text = true;
          this._text = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool text;
      }

      public removeKeyword_args() {
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
              if (field.Type == TType.String) {
                Text = iprot.ReadString();
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
        TStruct struc = new TStruct("removeKeyword_args");
        oprot.WriteStructBegin(struc);
        TField field = new TField();
        if (Text != null && __isset.text) {
          field.Name = "text";
          field.Type = TType.String;
          field.ID = 1;
          oprot.WriteFieldBegin(field);
          oprot.WriteString(Text);
          oprot.WriteFieldEnd();
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("removeKeyword_args(");
        sb.Append("Text: ");
        sb.Append(Text);
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getKeywords_args : TBase
    {

      public getKeywords_args() {
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
            default: 
              TProtocolUtil.Skip(iprot, field.Type);
              break;
          }
          iprot.ReadFieldEnd();
        }
        iprot.ReadStructEnd();
      }

      public void Write(TProtocol oprot) {
        TStruct struc = new TStruct("getKeywords_args");
        oprot.WriteStructBegin(struc);
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getKeywords_args(");
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getKeywords_result : TBase
    {
      private THashSet<string> _success;

      public THashSet<string> Success
      {
        get
        {
          return _success;
        }
        set
        {
          __isset.success = true;
          this._success = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool success;
      }

      public getKeywords_result() {
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
            case 0:
              if (field.Type == TType.Set) {
                {
                  Success = new THashSet<string>();
                  TSet _set8 = iprot.ReadSetBegin();
                  for( int _i9 = 0; _i9 < _set8.Count; ++_i9)
                  {
                    string _elem10 = null;
                    _elem10 = iprot.ReadString();
                    Success.Add(_elem10);
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
        TStruct struc = new TStruct("getKeywords_result");
        oprot.WriteStructBegin(struc);
        TField field = new TField();

        if (this.__isset.success) {
          if (Success != null) {
            field.Name = "Success";
            field.Type = TType.Set;
            field.ID = 0;
            oprot.WriteFieldBegin(field);
            {
              oprot.WriteSetBegin(new TSet(TType.String, Success.Count));
              foreach (string _iter11 in Success)
              {
                oprot.WriteString(_iter11);
              }
              oprot.WriteSetEnd();
            }
            oprot.WriteFieldEnd();
          }
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getKeywords_result(");
        sb.Append("Success: ");
        sb.Append(Success);
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class setAngle_args : TBase
    {
      private int _angle;

      public int Angle
      {
        get
        {
          return _angle;
        }
        set
        {
          __isset.angle = true;
          this._angle = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool angle;
      }

      public setAngle_args() {
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
                Angle = iprot.ReadI32();
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
        TStruct struc = new TStruct("setAngle_args");
        oprot.WriteStructBegin(struc);
        TField field = new TField();
        if (__isset.angle) {
          field.Name = "angle";
          field.Type = TType.I32;
          field.ID = 1;
          oprot.WriteFieldBegin(field);
          oprot.WriteI32(Angle);
          oprot.WriteFieldEnd();
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("setAngle_args(");
        sb.Append("Angle: ");
        sb.Append(Angle);
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getAngle_args : TBase
    {

      public getAngle_args() {
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
            default: 
              TProtocolUtil.Skip(iprot, field.Type);
              break;
          }
          iprot.ReadFieldEnd();
        }
        iprot.ReadStructEnd();
      }

      public void Write(TProtocol oprot) {
        TStruct struc = new TStruct("getAngle_args");
        oprot.WriteStructBegin(struc);
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getAngle_args(");
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getAngle_result : TBase
    {
      private int _success;

      public int Success
      {
        get
        {
          return _success;
        }
        set
        {
          __isset.success = true;
          this._success = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool success;
      }

      public getAngle_result() {
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
            case 0:
              if (field.Type == TType.I32) {
                Success = iprot.ReadI32();
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
        TStruct struc = new TStruct("getAngle_result");
        oprot.WriteStructBegin(struc);
        TField field = new TField();

        if (this.__isset.success) {
          field.Name = "Success";
          field.Type = TType.I32;
          field.ID = 0;
          oprot.WriteFieldBegin(field);
          oprot.WriteI32(Success);
          oprot.WriteFieldEnd();
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getAngle_result(");
        sb.Append("Success: ");
        sb.Append(Success);
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getFrame_args : TBase
    {

      public getFrame_args() {
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
            default: 
              TProtocolUtil.Skip(iprot, field.Type);
              break;
          }
          iprot.ReadFieldEnd();
        }
        iprot.ReadStructEnd();
      }

      public void Write(TProtocol oprot) {
        TStruct struc = new TStruct("getFrame_args");
        oprot.WriteStructBegin(struc);
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getFrame_args(");
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class getFrame_result : TBase
    {
      private Frame _success;

      public Frame Success
      {
        get
        {
          return _success;
        }
        set
        {
          __isset.success = true;
          this._success = value;
        }
      }


      public Isset __isset;
      #if !SILVERLIGHT
      [Serializable]
      #endif
      public struct Isset {
        public bool success;
      }

      public getFrame_result() {
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
            case 0:
              if (field.Type == TType.Struct) {
                Success = new Frame();
                Success.Read(iprot);
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
        TStruct struc = new TStruct("getFrame_result");
        oprot.WriteStructBegin(struc);
        TField field = new TField();

        if (this.__isset.success) {
          if (Success != null) {
            field.Name = "Success";
            field.Type = TType.Struct;
            field.ID = 0;
            oprot.WriteFieldBegin(field);
            Success.Write(oprot);
            oprot.WriteFieldEnd();
          }
        }
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("getFrame_result(");
        sb.Append("Success: ");
        sb.Append(Success== null ? "<null>" : Success.ToString());
        sb.Append(")");
        return sb.ToString();
      }

    }


    #if !SILVERLIGHT
    [Serializable]
    #endif
    public partial class stop_args : TBase
    {

      public stop_args() {
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
            default: 
              TProtocolUtil.Skip(iprot, field.Type);
              break;
          }
          iprot.ReadFieldEnd();
        }
        iprot.ReadStructEnd();
      }

      public void Write(TProtocol oprot) {
        TStruct struc = new TStruct("stop_args");
        oprot.WriteStructBegin(struc);
        oprot.WriteFieldStop();
        oprot.WriteStructEnd();
      }

      public override string ToString() {
        StringBuilder sb = new StringBuilder("stop_args(");
        sb.Append(")");
        return sb.ToString();
      }

    }

  }
}
