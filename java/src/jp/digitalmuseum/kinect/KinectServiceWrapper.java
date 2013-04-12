package jp.digitalmuseum.kinect;

import java.awt.Graphics;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.DataInputStream;
import java.io.DataOutputStream;
import java.io.IOException;
import java.net.InetSocketAddress;
import java.net.Socket;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.nio.IntBuffer;
import java.nio.ShortBuffer;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import org.apache.thrift.TException;
import org.apache.thrift.protocol.TBinaryProtocol;
import org.apache.thrift.protocol.TProtocol;
import org.apache.thrift.transport.TSocket;
import org.apache.thrift.transport.TTransport;
import org.apache.thrift.transport.TTransportException;

public class KinectServiceWrapper implements KinectService.Iface {
	private TTransport transport;
	private KinectService.Client client;
	private Frame frame;
	private BufferedImage image;
	private ScheduledExecutorService ses;
	private Future<?> future;
	private Set<FrameListener> listeners;

	public KinectServiceWrapper(String host, int port) {
		this(host, port, 300);
	}

	public KinectServiceWrapper(String host, int port, int timeout) {
		transport = new TSocket(host, port, timeout);
		TProtocol protocol = new TBinaryProtocol(transport);
		client = new KinectService.Client(protocol);
		image = new BufferedImage(
				640, 480, BufferedImage.TYPE_INT_RGB);
		listeners = new HashSet<FrameListener>();
	}
	
	public static final int playerIndexBitmask = 7;
	public static final int playerIndexBitmaskWidth = 3;
	
	// @see http://msdn.microsoft.com/en-us/library/hh973078.aspx#Depth_Ranges
	public static final short tooNearDepth = 0x0000;
	public static final short tooFarDepth = 0x0fff;
	public static final short unknownDepth = 0x1fff;
	
	public static void drawSkeleton(Graphics g, Map<JointType, Joint> joints, int x, int y) {
		drawSkeleton(g, joints, x, y, 1);
	}
	public static void drawSkeleton(Graphics g, Map<JointType, Joint> joints, int x, int y, float scale) {
		if (joints != null
				&& joints.size() == 20) {
			drawLine(g, x, y, scale, joints,
					JointType.HIP_CENTER,
					JointType.SPINE,
					JointType.SHOULDER_CENTER,
					JointType.HEAD);
			drawLine(g, x, y, scale, joints,
					JointType.SHOULDER_CENTER,
					JointType.SHOULDER_RIGHT,
					JointType.ELBOW_RIGHT,
					JointType.WRIST_RIGHT,
					JointType.HAND_RIGHT);
			drawLine(g, x, y, scale, joints,
					JointType.SHOULDER_CENTER,
					JointType.SHOULDER_LEFT,
					JointType.ELBOW_LEFT,
					JointType.WRIST_LEFT,
					JointType.HAND_LEFT);
			drawLine(g, x, y, scale, joints,
					JointType.HIP_CENTER,
					JointType.HIP_RIGHT,
					JointType.KNEE_RIGHT,
					JointType.ANKLE_RIGHT,
					JointType.FOOT_RIGHT);
			drawLine(g, x, y, scale, joints,
					JointType.HIP_CENTER,
					JointType.HIP_LEFT,
					JointType.KNEE_LEFT,
					JointType.ANKLE_LEFT,
					JointType.FOOT_LEFT);
		}
	}
	
	private static void drawLine(Graphics g, int x, int y, float scale, Map<JointType, Joint> joints, JointType... keys) {
		Joint sj = joints.get(keys[0]);
		double sx = 0, sy = 0;
		if (sj != null) {
			sx = sj.screenPosition.x * scale + x;
			sy = sj.screenPosition.y * scale + y;
		}
		for (int i = 1; i < keys.length; i ++) {
			Joint ej = joints.get(keys[i]);
			double ex = 0, ey = 0;
			if (ej != null) {
				ex = ej.screenPosition.x * scale + x;
				ey = ej.screenPosition.y * scale + y;
			}
			if (sj != null && ej != null) {
				g.drawLine((int)sx, (int)sy, (int)ex, (int)ey);
			}
			sj = ej;
			sx = ex;
			sy = ey;
		}
	}
	
	public static int getPlayerIndex(short depthValue) {
		return depthValue & playerIndexBitmask;
	}
	
	public static int getRealDepth(short depthValue) {
		return depthValue >> playerIndexBitmaskWidth;
	}
	
	public synchronized boolean start() {
		try {
			transport.open();
		} catch (TTransportException e) {
			return false;
		}
		if (ses != null) {
			ses.shutdown();
		}
		ses = Executors.newSingleThreadScheduledExecutor();
		future = ses.scheduleAtFixedRate(
				new FrameGrabber(), 0, 33, TimeUnit.MILLISECONDS);
		return true;
	}
	
	public synchronized void stop() {
		if (future != null) {
			future.cancel(true);
			transport.close();
			future = null;
		}
		if (ses != null) {
			ses.shutdown();
		}
	}
	
	public synchronized boolean isStarted() {
		return future != null;
	}
	
	public synchronized void addFrameListener(FrameListener listener) {
		this.listeners.add(listener);
	}
	
	public synchronized boolean removeFrameListener(FrameListener listener) {
		return this.listeners.remove(listener);
	}
	
	public interface FrameListener {
		public void frameUpdated(Frame frame, BufferedImage image, short[] depthImageData);
	}
	
	private class FrameGrabber implements Runnable {
		private Socket socket;
		private DataInputStream dis;
		private DataOutputStream dos;

		private DataBufferInt colorImageBuffer;
		private IntBuffer colorIntBuffer;
		private ByteBuffer byteBuffer;
		private ByteBuffer bb;
		byte[] buffer;

		public FrameGrabber() {
			connect();
			colorImageBuffer = (DataBufferInt) image.getRaster().getDataBuffer();
			colorIntBuffer = IntBuffer.wrap(colorImageBuffer.getData());

			buffer = new byte[640 * 480 * 4];
			byteBuffer = ByteBuffer.wrap(buffer);
			bb = ByteBuffer.allocate(8);

			// C# server running on Windows converts short[] to byte[] with little-endian.
			// Therefore, we need to specify the endian-ness here to reconstruct it correctly.
			byteBuffer.order(ByteOrder.LITTLE_ENDIAN);
			bb.order(ByteOrder.LITTLE_ENDIAN);
		}

		public void run() {
			if (!socket.isConnected()) {
				if (!connect()) {
					return;
				}
			}
			try {
				// Request info.
				bb.putChar('c');
				bb.rewind();
				dos.writeByte(bb.get());
				dos.writeByte(bb.get());
				dos.flush();
				bb.rewind();
				
				// Receive info.
				//  char: header
				bb.put(dis.readByte());
				bb.put(dis.readByte());
				bb.rewind();
				char c = bb.getChar();
				bb.rewind();

				//  int: image length
				for (int i = 0; i < 4; i ++) {
					bb.put(dis.readByte());
				}
				bb.rewind();
				int len = bb.getInt();

				//  byte: skeleton availability
				byte b = dis.readByte();
				boolean isSkeletonAvailable = b == 1;

				//  int[]: image data
				int read = 0;
				while (true) {
					int r = dis.read(buffer, read, len - read);
					read += r;
					if (r == 0 || read == len) {
						break;
					}
				}
				byteBuffer.rewind();

				if (c == 'c') {
					colorIntBuffer.clear();
					colorIntBuffer.put(byteBuffer.asIntBuffer());
				} else if (c == 'd') {
					//
				}
				
				HashMap<JointType,Joint> jointMap = new HashMap<JointType,Joint>();
				Frame frame = new Frame(0, jointMap);
				if (isSkeletonAvailable) {
					//
					for (int i = 0; i < 4; i ++) {
						bb.put(dis.readByte());
					}
					bb.rewind();
					int numJoints = bb.getInt();

					for (int i = 0; i < numJoints; i ++) {

						for (int j = 0; j < 4; j ++) {
							bb.put(dis.readByte());
						}
						bb.rewind();
						JointType jt = JointType.findByValue(bb.getInt());

						double[] coords = new double[5];
						for (int k = 0; k < 5; k ++) {
							for (int j = 0; j < 8; j ++) {
								bb.put(dis.readByte());
							}
							bb.rewind();
							coords[k] = bb.getDouble();
						}
						Joint joint = new Joint();
						joint.position = new Position3D(coords[0], coords[1], coords[2]);
						joint.screenPosition = new Position2D(coords[3], coords[4]);

						jointMap.put(jt, joint);
					}
				}

				for (FrameListener listener : listeners) {
					listener.frameUpdated(frame, image, null);
				}

			} catch (IOException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
				try {
					socket.close();
				} catch (IOException e1) {
					e1.printStackTrace();
				}
			}
			
		}
		
		private boolean connect() {
			try {
				if (socket == null) {
					socket = new Socket();
				}
				socket.connect(new InetSocketAddress("127.0.0.1", KinectServiceConstants.SERVER_DEFAULT_PORT + 1), 1000);
				dis = new DataInputStream(socket.getInputStream());
				dos = new DataOutputStream(socket.getOutputStream());
				return true;
			} catch (Exception e) {
				e.printStackTrace();
				return false;
			}
		}
	}

	@Override
	public synchronized boolean isDeviceConnected() throws TException {
		return client.isDeviceConnected();
	}
	
	@Override
	public synchronized void setVoiceEnabled(boolean isEnabled) throws TException {
		client.setVoiceEnabled(isEnabled);
	}

	@Override
	public synchronized boolean isVoiceEnabled() throws TException {
		return client.isVoiceEnabled();
	}

	@Override
	public synchronized void addKeyword(String text) throws TException {
		client.addKeyword(text);
	}

	@Override
	public synchronized void removeKeyword(String text) throws TException {
		client.removeKeyword(text);
	}

	@Override
	public synchronized void setColorEnabled(boolean isEnabled) throws TException {
		client.setColorEnabled(isEnabled);
	}

	@Override
	public synchronized boolean isColorEnabled() throws TException {
		return client.isColorEnabled();
	}
	
	@Override
	public synchronized void setDepthEnabled(boolean isEnabled) throws TException {
		client.setDepthEnabled(isEnabled);
	}

	@Override
	public synchronized boolean isDepthEnabled() throws TException {
		return client.isDepthEnabled();
	}

	@Override
	public synchronized void setAngle(int angle) throws TException {
		client.setAngle(angle);
	}

	@Override
	public synchronized int getAngle() throws TException {
		return client.getAngle();
	}

	@Override
	public Frame getFrame() {
		return frame;
	}

	@Override
	public synchronized void shutdown() throws TException {
		client.shutdown();
	}
}
