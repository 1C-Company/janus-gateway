/*! \file   janus_videoroom.c
 * \author Lorenzo Miniero <lorenzo@meetecho.com>
 * \copyright GNU General Public License v3
 * \brief  Janus VideoRoom plugin
 * \details Check the \ref videoroom for more details.
 *
 * \ingroup plugins
 * \ref plugins
 *
 * \page videoroom VideoRoom plugin documentation
 * This is a plugin implementing a videoconferencing SFU
 * (Selective Forwarding Unit) for Janus, that is an audio/video router.
 * This means that the plugin implements a virtual conferencing room peers
 * can join and leave at any time. This room is based on a Publish/Subscribe
 * pattern. Each peer can publish his/her own live audio/video feeds: this
 * feed becomes an available stream in the room the other participants can
 * subscribe to. This means that this plugin allows the realization of several
 * different scenarios, ranging from a simple webinar (one speaker, several
 * watchers) to a fully meshed video conference (each peer sending and
 * receiving to and from all the others).
 *
 * Notice that, since Janus now supports multistream PeerConnections,
 * subscriptions can be done either in "bulks" (you use a single PeerConnection
 * to subscribe to multiple streams from one or more publishers) or
 * separately (each PeerConnections represents a subscription to a single
 * publisher). Same thing for publishers: you may choose to publish, e.g.,
 * audio and video on one PeerConnection, and share your screen on another,
 * or publish everything on the same PeerConnection instead. While
 * functionally both approaches (multistream vs. legacy mode) are the same
 * (the same media flows in both cases), the differences are in how
 * resources are used, and in how the client has to handle incoming and
 * outgoing connections. Besides, one approach might make more sense in
 * some scenarios, and the other make more sense in different use cases.
 * As such, the approach to follow is left to the developer and the application.
 *
 * What is important to point out, though, is that publishers and subscribers
 * will in all cases require different PeerConnections. This means that,
 * even with multistream, you won't be able to use a single PeerConnection
 * to send your contributions and receive those from everyone else. This
 * is a choice done by design, to avoid the issues that would inevitably
 * arise when doing, for instance, renegotiations to update the streams.
 *
 * On a more general note and to give some more context with respect to the
 * core functionality in Janus, notice that, considering this plugin allows
 * for several different WebRTC PeerConnections to be on at the same time
 * for the same peer (different publishers and subscribers for sure, and
 * potentially more than one of each if multistream is not in use), each
 * peer will often need to attach several times to the same plugin for each
 * stream: this means that each peer needs to have at least one handle active
 * for managing its relation with the plugin (joining a room,
 * leaving a room, muting/unmuting, publishing, receiving events), and needs
 * to open others when they want to subscribe to a feed from other participants
 * (the number depends on the subscription approach of choice). Handles
 * used for subscriptions, though, would be logically "subjects" to the
 * master one used for managing the room: this means that they cannot be
 * used, for instance, to unmute in the room, as their only purpose would
 * be to provide a context in which creating the recvonly PeerConnections
 * for the subscription(s).
 *
 * Rooms to make available are listed in the plugin configuration file.
 * A pre-filled configuration file is provided in \c conf/janus.plugin.videoroom.jcfg
 * and includes a demo room for testing. The same plugin is also used
 * dynamically (that is, with rooms created on the fly via API) in the
 * Screen Sharing demo as well.
 *
 * To add more rooms or modify the existing one, you can use the following
 * syntax:
 *
 * \verbatim
room-<unique room ID>: {
	description = This is my awesome room
	is_private = true|false (private rooms don't appear when you do a 'list' request, default=false)
	secret = <optional password needed for manipulating (e.g. destroying) the room>
	pin = <optional password needed for joining the room>
	require_pvtid = true|false (whether subscriptions are required to provide a valid private_id
				 to associate with a publisher, default=false)
	signed_tokens = true|false (whether access to the room requires signed tokens; default=false,
				 only works if signed tokens are used in the core as well)
	publishers = <max number of concurrent senders> (e.g., 6 for a video
				 conference or 1 for a webinar, default=3)
	bitrate = <max video bitrate for senders> (e.g., 128000)
	bitrate_cap = <true|false, whether the above cap should act as a limit to dynamic bitrate changes by publishers, default=false>,
	fir_freq = <send a FIR to publishers every fir_freq seconds> (0=disable)
	audiocodec = opus|g722|pcmu|pcma|isac32|isac16 (audio codec to force on publishers, default=opus
				can be a comma separated list in order of preference, e.g., opus,pcmu)
	videocodec = vp8|vp9|h264|av1|h265 (video codec to force on publishers, default=vp8
				can be a comma separated list in order of preference, e.g., vp9,vp8,h264)
	vp9_profile = VP9-specific profile to prefer (e.g., "2" for "profile-id=2")
	h264_profile = H.264-specific profile to prefer (e.g., "42e01f" for "profile-level-id=42e01f")
	opus_fec = true|false (whether inband FEC must be negotiated; only works for Opus, default=true)
	opus_dtx = true|false (whether DTX must be negotiated; only works for Opus, default=false)
	video_svc = true|false (whether SVC support must be enabled; only works for VP9, default=false)
	audiolevel_ext = true|false (whether the ssrc-audio-level RTP extension must be
		negotiated/used or not for new publishers, default=true)
	audiolevel_event = true|false (whether to emit event to other users or not, default=false)
	audio_active_packets = 100 (number of packets with audio level, default=100, 2 seconds)
	audio_level_average = 25 (average value of audio level, 127=muted, 0='too loud', default=25)
	videoorient_ext = true|false (whether the video-orientation RTP extension must be
		negotiated/used or not for new publishers, default=true)
	playoutdelay_ext = true|false (whether the playout-delay RTP extension must be
		negotiated/used or not for new publishers, default=true)
	transport_wide_cc_ext = true|false (whether the transport wide CC RTP extension must be
		negotiated/used or not for new publishers, default=true)
	record = true|false (whether this room should be recorded, default=false)
	rec_dir = <folder where recordings should be stored, when enabled>
	lock_record = true|false (whether recording can only be started/stopped if the secret
				is provided, or using the global enable_recording request, default=false)
	notify_joining = true|false (optional, whether to notify all participants when a new
				participant joins the room. The Videoroom plugin by design only notifies
				new feeds (publishers), and enabling this may result extra notification
				traffic. This flag is particularly useful when enabled with require_pvtid
				for admin to manage listening only participants. default=false)
	require_e2ee = true|false (whether all participants are required to publish and subscribe
				using end-to-end media encryption, e.g., via Insertable Streams; default=false)
	dummy_publisher = true|false (whether a dummy publisher should be created in this room,
				with one separate m-line for each codec supported in the room; this is
				useful when there's a need to create subscriptions with placeholders
				for some or all m-lines, even when they aren't used yet; default=false)
	dummy_streams = in case dummy_publisher is set to true, array of codecs to offer,
				optionally with a fmtp attribute to match (codec/fmtp properties).
				If not provided, all codecs enabled in the room are offered, with no fmtp.
				Notice that the fmtp is parsed, and only a few codecs are supported.
}
\endverbatim
 *
 * Note that recording will work with all codecs except iSAC.
 *
 * \section sfuapi Video Room API
 *
 * The Video Room API supports several requests, some of which are
 * synchronous and some asynchronous. There are some situations, though,
 * (invalid JSON, invalid request) which will always result in a
 * synchronous error response even for asynchronous requests.
 *
 * \c create , \c destroy , \c edit , \c exists, \c list, \c allowed,
 * \c kick , \c moderate , \c enable_recording , \c listparticipants
 * and \c listforwarders are synchronous requests, which means you'll
 * get a response directly within the context of the transaction.
 * \c create allows you to create a new video room dynamically, as an
 * alternative to using the configuration file; \c edit allows you to
 * dynamically edit some room properties (e.g., the PIN); \c destroy removes a
 * video room and destroys it, kicking all the users out as part of the
 * process; \c exists allows you to check whether a specific video room
 * exists; finally, \c list lists all the available rooms, while \c
 * listparticipants lists all the active (as in currently publishing
 * something) participants of a specific room and their details.
 *
 * The \c join , \c joinandconfigure , \c configure , \c publish ,
 * \c unpublish , \c start , \c pause , \c switch and \c leave
 * requests instead are all asynchronous, which
 * means you'll get a notification about their success or failure in
 * an event. \c join allows you to join a specific video room, specifying
 * whether that specific PeerConnection will be used for publishing or
 * watching; \c configure can be used to modify some of the participation
 * settings (e.g., bitrate cap); \c joinandconfigure combines the previous
 * two requests in a single one (just for publishers); \c publish can be
 * used to start sending media to broadcast to the other participants,
 * while \c unpublish does the opposite; \c start allows you to start
 * receiving media from a publisher you've subscribed to previously by
 * means of a \c join , while \c pause pauses the delivery of the media;
 * the \c switch request can be used to change the source of the media
 * flowing over a specific PeerConnection (e.g., I was watching Alice,
 * I want to watch Bob now) without having to create a new handle for
 * that; finally, \c leave allows you to leave a video room for good
 * (or, in the case of viewers, definitely closes a subscription).
 *
 * \c create can be used to create a new video room, and has to be
 * formatted as follows:
 *
\verbatim
{
	"request" : "create",
	"room" : <unique numeric ID, optional, chosen by plugin if missing>,
	"permanent" : <true|false, whether the room should be saved in the config file, default=false>,
	"description" : "<pretty name of the room, optional>",
	"secret" : "<password required to edit/destroy the room, optional>",
	"pin" : "<password required to join the room, optional>",
	"is_private" : <true|false, whether the room should appear in a list request>,
	"allowed" : [ array of string tokens users can use to join this room, optional],
	...
}
\endverbatim
 *
 * For the sake of brevity, not all of the available settings are listed
 * here. You can refer to the name of the properties in the configuration
 * file as a reference, as the ones used to programmatically create a new
 * room are exactly the same.
 *
 * A successful creation procedure will result in a \c created response:
 *
\verbatim
{
	"videoroom" : "created",
	"room" : <unique numeric ID>,
	"permanent" : <true if saved to config file, false if not>
}
\endverbatim
 *
 * If you requested a permanent room but a \c false value is returned
 * instead, good chances are that there are permission problems.
 *
 * An error instead (and the same applies to all other requests, so this
 * won't be repeated) would provide both an error code and a more verbose
 * description of the cause of the issue:
 *
\verbatim
{
	"videoroom" : "event",
	"error_code" : <numeric ID, check Macros below>,
	"error" : "<error description as a string>"
}
\endverbatim
 *
 * Notice that, in general, all users can create rooms. If you want to
 * limit this functionality, you can configure an admin \c admin_key in
 * the plugin settings. When configured, only "create" requests that
 * include the correct \c admin_key value in an "admin_key" property
 * will succeed, and will be rejected otherwise. Notice that you can
 * optionally extend this functionality to RTP forwarding as well, in
 * order to only allow trusted clients to use that feature.
 *
 * Once a room has been created, you can still edit some (but not all)
 * of its properties using the \c edit request. This allows you to modify
 * the room description, secret, pin and whether it's private or not: you
 * won't be able to modify other more static properties, like the room ID,
 * the sampling rate, the extensions-related stuff and so on. If you're
 * interested in changing the ACL, instead, check the \c allowed message.
 * An \c edit request has to be formatted as follows:
 *
\verbatim
{
	"request" : "edit",
	"room" : <unique numeric ID of the room to edit>,
	"secret" : "<room secret, mandatory if configured>",
	"new_description" : "<new pretty name of the room, optional>",
	"new_secret" : "<new password required to edit/destroy the room, optional>",
	"new_pin" : "<new password required to join the room, optional>",
	"new_is_private" : <true|false, whether the room should appear in a list request>,
	"new_require_pvtid" : <true|false, whether the room should require private_id from subscribers>,
	"new_bitrate" : <new bitrate cap to force on all publishers (except those with custom overrides)>,
	"new_fir_freq" : <new period for regular PLI keyframe requests to publishers>,
	"new_publishers" : <new cap on the number of concurrent active WebRTC publishers>,
	"new_lock_record" : <true|false, whether recording state can only be changed when providing the room secret>,
	"new_rec_dir" : "<the new path where the next .mjr files should being saved>",
	"permanent" : <true|false, whether the room should be also removed from the config file, default=false>
}
\endverbatim
 *
 * A successful edit procedure will result in an \c edited response:
 *
\verbatim
{
	"videoroom" : "edited",
	"room" : <unique numeric ID>
}
\endverbatim
 *
 * On the other hand, \c destroy can be used to destroy an existing video
 * room, whether created dynamically or statically, and has to be
 * formatted as follows:
 *
\verbatim
{
	"request" : "destroy",
	"room" : <unique numeric ID of the room to destroy>,
	"secret" : "<room secret, mandatory if configured>",
	"permanent" : <true|false, whether the room should be also removed from the config file, default=false>
}
\endverbatim
 *
 * A successful destruction procedure will result in a \c destroyed response:
 *
\verbatim
{
	"videoroom" : "destroyed",
	"room" : <unique numeric ID>
}
\endverbatim
 *
 * This will also result in a \c destroyed event being sent to all the
 * participants in the video room, which will look like this:
 *
\verbatim
{
	"videoroom" : "destroyed",
	"room" : <unique numeric ID of the destroyed room>
}
\endverbatim
 *
 * You can check whether a room exists using the \c exists request,
 * which has to be formatted as follows:
 *
\verbatim
{
	"request" : "exists",
	"room" : <unique numeric ID of the room to check>
}
\endverbatim
 *
 * A successful request will result in a \c success response:
 *
\verbatim
{
	"videoroom" : "success",
	"room" : <unique numeric ID>,
	"exists" : <true|false>
}
\endverbatim
 *
 * You can configure whether to check tokens or add/remove people who can join
 * a room using the \c allowed request, which has to be formatted as follows:
 *
\verbatim
{
	"request" : "allowed",
	"secret" : "<room secret, mandatory if configured>",
	"action" : "enable|disable|add|remove",
	"room" : <unique numeric ID of the room to update>,
	"allowed" : [
		// Array of strings (tokens users might pass in "join", only for add|remove)
	]
}
\endverbatim
 *
 * A successful request will result in a \c success response:
 *
\verbatim
{
	"videoroom" : "success",
	"room" : <unique numeric ID>,
	"allowed" : [
		// Updated, complete, list of allowed tokens (only for enable|add|remove)
	]
}
\endverbatim
 *
 * If you're the administrator of a room (that is, you created it and have access
 * to the secret) you can kick participants using the \c kick request. Notice
 * that this only kicks the user out of the room, but does not prevent them from
 * re-joining: to ban them, you need to first remove them from the list of
 * authorized users (see \c allowed request) and then \c kick them. The \c kick
 * request has to be formatted as follows:
 *
\verbatim
{
	"request" : "kick",
	"secret" : "<room secret, mandatory if configured>",
	"room" : <unique numeric ID of the room>,
	"id" : <unique numeric ID of the participant to kick>
}
\endverbatim
 *
 * A successful request will result in a \c success response:
 *
\verbatim
{
	"videoroom" : "success",
}
\endverbatim
 *
 * As an administrator, you can also forcibly mute/unmute any of the media
 * streams sent by participants (i.e., audio, video and data streams),
 * using the \c moderate requests. Notice that if the participant is self
 * muted on a stream, and you unmute that stream with \c moderate, they
 * will NOT be unmuted: you'll simply remove any moderation block
 * that may have been enforced on the participant for that medium
 * themselves. The \c moderate request has to be formatted as follows:
 *
\verbatim
{
	"request" : "moderate",
	"secret" : "<room secret, mandatory if configured>",
	"room" : <unique numeric ID of the room>,
	"id" : <unique numeric ID of the participant to moderate>,
	"mid" : <mid of the m-line to refer to for this moderate request>,
	"mute" : <true|false, depending on whether the media addressed by the above mid should be muted by the moderator>
}
\endverbatim
 *
 * A successful request will result in a \c success response:
 *
\verbatim
{
	"videoroom" : "success",
}
\endverbatim
 *
 * To get a list of the available rooms you can make use of the \c list request.
 * \c admin_key is optional. If included and correct, rooms configured/created
 * as private will be included in the list as well.
 *
\verbatim
{
	"request" : "list"
}
\endverbatim
 *
 * A successful request will produce a list of rooms in a \c success response:
 *
\verbatim
{
	"videoroom" : "success",
	"list" : [		// Array of room objects
		{	// Room #1
			"room" : <unique numeric ID>,
			"description" : "<Name of the room>",
			"pin_required" : <true|false, whether a PIN is required to join this room>,
			"is_private" : <true|false, whether this room is 'private' (as in hidden) or not>,
			"max_publishers" : <how many publishers can actually publish via WebRTC at the same time>,
			"bitrate" : <bitrate cap that should be forced (via REMB) on all publishers by default>,
			"bitrate_cap" : <true|false, whether the above cap should act as a limit to dynamic bitrate changes by publishers (optional)>,
			"fir_freq" : <how often a keyframe request is sent via PLI/FIR to active publishers>,
			"require_pvtid": <true|false, whether subscriptions in this room require a private_id>,
			"require_e2ee": <true|false, whether end-to-end encrypted publishers are required>,
			"dummy_publisher": <true|false, whether a dummy publisher exists for placeholder subscriptions>,
			"notify_joining": <true|false, whether an event is sent to notify all participants if a new participant joins the room>,
			"audiocodec" : "<comma separated list of allowed audio codecs>",
			"videocodec" : "<comma separated list of allowed video codecs>",
			"opus_fec": <true|false, whether inband FEC must be negotiated (note: only available for Opus) (optional)>,
			"opus_dtx": <true|false, whether DTX must be negotiated (note: only available for Opus) (optional)>,
			"video_svc": <true|false, whether SVC must be done for video (note: only available for VP9 right now) (optional)>,
			"record" : <true|false, whether the room is being recorded>,
			"rec_dir" : "<if recording, the path where the .mjr files are being saved>",
			"lock_record" : <true|false, whether the room recording state can only be changed providing the secret>,
			"num_participants" : <count of the participants (publishers, active or not; not subscribers)>
			"audiolevel_ext": <true|false, whether the ssrc-audio-level extension must be negotiated or not for new publishers>,
			"audiolevel_event": <true|false, whether to emit event to other users about audiolevel>,
			"audio_active_packets": <amount of packets with audio level for checkup (optional, only if audiolevel_event is true)>,
			"audio_level_average": <average audio level (optional, only if audiolevel_event is true)>,
			"videoorient_ext": <true|false, whether the video-orientation extension must be negotiated or not for new publishers>,
			"playoutdelay_ext": <true|false, whether the playout-delay extension must be negotiated or not for new publishers>,
			"transport_wide_cc_ext": <true|false, whether the transport wide cc extension must be negotiated or not for new publishers>
		},
		// Other rooms
	]
}
\endverbatim
 *
 * To get a list of the participants in a specific room, instead, you
 * can make use of the \c listparticipants request, which has to be
 * formatted as follows:
 *
\verbatim
{
	"request" : "listparticipants",
	"room" : <unique numeric ID of the room>
}
\endverbatim
 *
 * A successful request will produce a list of participants in a
 * \c participants response:
 *
\verbatim
{
	"videoroom" : "participants",
	"room" : <unique numeric ID of the room>,
	"participants" : [		// Array of participant objects
		{	// Participant #1
			"id" : <unique numeric ID of the participant>,
			"display" : "<display name of the participant, if any; optional>",
			"publisher" : "<true|false, whether user is an active publisher in the room>",
			"talking" : <true|false, whether user is talking or not (only if audio levels are used)>
		},
		// Other participants
	]
}
\endverbatim
 *
 * This covers almost all the synchronous requests. All the asynchronous requests,
 * plus a couple of additional synchronous requests we'll cover later, refer
 * to participants instead, namely on how they can publish, subscribe, or
 * more in general manage the media streams they may be sending or receiving.
 *
 * Considering the different nature of publishers and subscribers in the room,
 * and more importantly how you establish PeerConnections in the respective
 * cases, their API requests are addressed in separate subsections.
 *
 * \subsection vroompub VideoRoom Publishers
 *
 * In a VideoRoom, publishers are those participant handles that are able
 * (although may choose not to, more on this later) publish media in the
 * room, and as such become feeds that you can subscribe to.
 *
 * To specify that a handle will be associated with a publisher, you must use
 * the \c join request with \c ptype set to \c publisher (note that, as it
 * will be explained later, you can also use \c joinandconfigure for the
 * purpose). The exact syntax of the request is the following:
 *
\verbatim
{
	"request" : "join",
	"ptype" : "publisher",
	"room" : <unique ID of the room to join>,
	"id" : <unique ID to register for the publisher; optional, will be chosen by the plugin if missing>,
	"display" : "<display name for the publisher; optional>",
	"token" : "<invitation token, in case the room has an ACL; optional>"
}
\endverbatim
 *
 * This will add the user to the list of participants in the room, although
 * in a non-active role for the time being. Anyway, this participation
 * allows the user to receive notifications about several aspects of the
 * room on the related handle (including streams as they become available
 * and go away). As such, it can be used even just as a way to get
 * notifications in a room, without the need of ever actually publishing
 * any stream at all (which explains why the "publisher" role may actually
 * be a bit confusing in this context).
 *
 * A successful \c join will result in a \c joined event, which will contain
 * a list of the currently active (as in publishing via WebRTC) publishers,
 * and optionally a list of passive attendees (but only if the room was
 * configured with \c notify_joining set to \c TRUE ):
 *
\verbatim
{
	"videoroom" : "joined",
	"room" : <room ID>,
	"description" : <description of the room, if available>,
	"id" : <unique ID of the participant>,
	"private_id" : <a different unique ID associated to the participant; meant to be private>,
	"publishers" : [
		{
			"id" : <unique ID of active publisher #1>,
			"display" : "<display name of active publisher #1, if any>",
			"dummy" : <true if this participant is a dummy publisher>,
			"streams" : [
				{
					"type" : "<type of published stream #1 (audio|video|data)">,
					"mindex" : "<unique mindex of published stream #1>",
					"mid" : "<unique mid of of published stream #1>",
					"disabled" : <if true, it means this stream is currently inactive/disabled (and so codec, description, etc. will be missing)>,
					"codec" : "<codec used for published stream #1>",
					"description" : "<text description of published stream #1, if any>",
					"moderated" : <true if this stream audio has been moderated for this participant>,
					"simulcast" : "<true if published stream #1 uses simulcast (VP8 and H.264 only)>",
					"svc" : "<true if published stream #1 uses SVC (VP9 only)>",
					"talking" : <true|false, whether the publisher stream has audio activity or not (only if audio levels are used)>,
				},
				// Other streams, if any
			],
			"talking" : <true|false, whether the publisher is talking or not (only if audio levels are used); deprecated, use the stream specific ones>,
		},
		// Other active publishers
	],
	"attendees" : [		// Only present when notify_joining is set to TRUE for rooms
		{
			"id" : <unique ID of attendee #1>,
			"display" : "<display name of attendee #1, if any>"
		},
		// Other attendees
	]
}
\endverbatim
 *
 * Notice that the publishers list will of course be empty if no one is
 * currently active in the room. For what concerns the \c private_id
 * property, it is meant to be used by the user when they create subscriptions,
 * so that the plugin can associate subscriber handles (which are typically
 * anonymous) to a specific participant; they're usually optional, unless
 * required by the room configuration.
 *
 * As explained, with a simple \c join you're not an active publisher (there
 * is no WebRTC PeerConnection yet), which means that by default your presence
 * is not notified to other participants. In fact, the publish/subscribe nature
 * of the plugin implies that by default only active publishers are notified,
 * to allow participants to subscribe to existing feeds: notifying all joins/leaves,
 * even those related to who will just lurk, may be overly verbose and chatty,
 * especially in large rooms. Anyway, rooms can be configured to notify those
 * as well, if the \c notify_joining property is set to true: in that case,
 * regular joins will be notified too, in an event formatted like this:
 *
\verbatim
{
	"videoroom" : "event",
	"room" : <room ID>,
	"joining" : {
		"id" : <unique ID of the new participant>,
		"display" : "<display name of the new participant, if any>"
	}
}
\endverbatim
 *
 * If you're interested in publishing media within a room, you can do that
 * with a \c publish request. This request MUST be accompanied by a JSEP
 * SDP offer to negotiate a new PeerConnection. The plugin will match it
 * to the room configuration (e.g., to make sure the codecs you negotiated
 * are allowed in the room), and will reply with a JSEP SDP answer to
 * close the circle and complete the setup of the PeerConnection. As soon
 * as the PeerConnection has been established, the publisher will become
 * active, and a new active feed other participants can subscribe to.
 *
 * The syntax of a \c publish request is the following:
 *
\verbatim
{
	"request" : "publish",
	"audiocodec" : "<audio codec to prefer among the negotiated ones; optional>",
	"videocodec" : "<video codec to prefer among the negotiated ones; optional>",
	"bitrate" : <bitrate cap to return via REMB; optional, overrides the global room value if present>,
	"record" : <true|false, whether this publisher should be recorded or not; optional>,
	"filename" : "<if recording, the base path/file to use for the recording files; optional>",
	"display" : "<display name to use in the room; optional>",
	"audio_level_average" : "<if provided, overrided the room audio_level_average for this user; optional>",
	"audio_active_packets" : "<if provided, overrided the room audio_active_packets for this user; optional>",
	"descriptions" : [	// Optional
		{
			"mid" : "<unique mid of a stream being published>",
			"description" : "<text description of the stream (e.g., My front webcam)>"
		},
		// Other descriptions, if any
	]}
\endverbatim
 *
 * As anticipated, since this is supposed to be accompanied by a JSEP SDP
 * offer describing the publisher's media streams, the plugin will negotiate
 * and prepare a matching JSEP SDP answer. Notice that, in principle, all
 * published streams will be only identified by their unique \c mid and
 * by their type (e.g., audio or video). In case you want to provide more
 * information about the streams being published (e.g., to let other
 * participants know that the first video is a camera, while the second
 * video is a screen share), you can use the \c descriptions array for
 * the purpose: each object in the array can be used to add a text description
 * to associate to a specific mid, in order to help with the UI rendering.
 * The \c descriptions property is optional, so no text will be provided
 * by default: notice these descriptions can be updated dynamically via
 * \c configure requests.
 *
 * If successful, a \c configured event will be sent back, formatted like this:
 *
\verbatim
{
	"videoroom" : "event",
	"configured" : "ok"
}
\endverbatim
 *
 * This event will be accompanied by the prepared JSEP SDP answer.
 *
 * Notice that you can also use \c configure as a request instead of
 * \c publish to start publishing. The two are functionally equivalent
 * for publishing, but from a semantic perspective \c publish is the
 * right message to send when publishing. The \c configure request, as
 * it will be clearer later, can also be used to update some properties
 * of the publisher session: in this case the \c publish request can NOT
 * be used, as it can only be invoked to publish, and will fail if you're
 * already publishing something.
 *
 * As an additional note, notice that you can also join and publish in
 * a single request, which is useful in case you're not interested in
 * first join as a passive attendee and only later publish something,
 * but want to publish something right away. In this case you can use
 * the \c joinandconfigure request, which as you can imagine combines
 * the properties of both \c join and \c publish in a single request:
 * the response to a \c joinandconfigure will be a \c joined event, and
 * will again be accompanied by a JSEP SDP answer as usual.
 *
 * However you decided to publish something, as soon as the PeerConnection
 * setup succeeds and the publisher becomes active, an event is sent to
 * all the participants in the room with information on the new feed.
 * The event must contain an array with a single element, and be formatted like this:
 *
\verbatim
{
	"videoroom" : "event",
	"room" : <room ID>,
	"publishers" : [
		{
			"id" : <unique ID of the new publisher>,
			"display" : "<display name of the new publisher, if any>",
			"dummy" : <true if this participant is a dummy publisher>,
			"streams" : [
				{
					"type" : "<type of published stream #1 (audio|video|data)">,
					"mindex" : "<unique mindex of published stream #1>",
					"mid" : "<unique mid of of published stream #1>",
					"disabled" : <if true, it means this stream is currently inactive/disabled (and so codec, description, etc. will be missing)>,
					"codec" : "<codec used for published stream #1>",
					"description" : "<text description of published stream #1, if any>",
					"moderated" : <true if this stream audio has been moderated for this participant>,
					"simulcast" : "<true if published stream #1 uses simulcast (VP8 and H.264 only)>",
					"svc" : "<true if published stream #1 uses SVC (VP9 only)>",
					"talking" : <true|false, whether the publisher stream has audio activity or not (only if audio levels are used)>,
				},
				// Other streams, if any
			],
			"talking" : <true|false, whether the publisher is talking or not (only if audio levels are used); deprecated, use the stream specific ones>,
		}
	]
}
\endverbatim
 *
 * To stop publishing and tear down the related PeerConnection, you can
 * use the \c unpublish request, which requires no arguments as the context
 * is implicit:
 *
\verbatim
{
	"request" : "unpublish"
}
\endverbatim
 *
 * This will have the plugin tear down the PeerConnection, and remove the
 * publisher from the list of active streams. If successful, the response
 * will look like this:
 *
\verbatim
{
	"videoroom" : "event",
	"unpublished" : "ok"
}
\endverbatim
 *
 * As soon as the PeerConnection is gone, all the other participants will
 * also be notified about the fact that the stream is no longer available:
 *
\verbatim
{
	"videoroom" : "event",
	"room" : <room ID>,
	"unpublished" : <unique ID of the publisher who unpublished>
}
\endverbatim
 *
 * Notice that the same event will also be sent whenever the publisher
 * feed disappears for reasons other than an explicit \c unpublish , e.g.,
 * because the handle was closed or the user lost their connection.
 * Besides, notice that you can publish and unpublish multiple times
 * within the context of the same publisher handle.
 *
 * As anticipated above, you can use a request called \c configure to
 * tweak some of the properties of an active publisher session. This
 * request must be formatted as follows:
 *
\verbatim
{
	"request" : "configure",
	"bitrate" : <bitrate cap to return via REMB; optional, overrides the global room value if present (unless bitrate_cap is set)>,
	"keyframe" : <true|false, whether we should send this publisher a keyframe request>,
	"record" : <true|false, whether this publisher should be recorded or not; optional>,
	"filename" : "<if recording, the base path/file to use for the recording files; optional>",
	"display" : "<new display name to use in the room; optional>",
	"audio_active_packets" : "<new audio_active_packets to overwrite in the room one; optional>",
	"audio_level_average" : "<new audio_level_average to overwrite the room one; optional>",
	"streams" : [
		{
			"mid" : <mid of the m-line to tweak>,
			"keyframe" : <true|false, whether we should send this stream a keyframe request; optional>,
			"send" : <true|false, depending on whether the media addressed by the above mid should be relayed or not; optional>,
			"min_delay" : <minimum delay to enforce via the playout-delay RTP extension, in blocks of 10ms; optional>,
			"max_delay" : <maximum delay to enforce via the playout-delay RTP extension, in blocks of 10ms; optional>
		},
		// Other streams, if any
	],
	"descriptions" : [
		// Updated descriptions for the published streams; see "publish" for syntax; optional
	]
}
\endverbatim
 *
 * As you can see, it's basically the same properties as those listed for
 * \c publish , with the addition of a \c streams array that can be used
 * to tweak individual streams (which is not available when publishing
 * since in that case the stream doesn't exist yet). Notice that the
 * \c configure request can also be used in renegotiations, to provide
 * an updated SDP with changes to the published media. If successful,
 * a \c configured event will be sent back as before, formatted like this:
 *
\verbatim
{
	"videoroom" : "event",
	"configured" : "ok"
}
\endverbatim
 *
 * When configuring the room to request the ssrc-audio-level RTP extension,
 * ad-hoc events might be sent to all publishers if \c audiolevel_event is
 * set to true. These events will have the following format:
 *
\verbatim
{
	"videoroom" : <"talking"|"stopped-talking", whether the publisher started or stopped talking>,
	"room" : <unique numeric ID of the room the publisher is in>,
	"id" : <unique numeric ID of the publisher>,
	"audio-level-dBov-avg" : <average value of audio level, 127=muted, 0='too loud'>
}
\endverbatim
 *
 * An interesting feature VideoRoom publisher can take advantage of is
 * RTP forwarding. In fact, while the main purpose of this plugin is
 * getting media from WebRTC sources (publishers) and relaying it to
 * WebRTC destinations (subscribers), there are actually several use
 * cases and scenarios for making this media available to external,
 * notnecessarily WebRTC-compliant, components. These components may
 * benefit from having access to the RTP media sent by a publisher, e.g.,
 * for media processing, external recording, transcoding to other
 * technologies via other applications, scalability purposes or
 * whatever else makes sense in this context. This is made possible by
 * a request called \c rtp_forward which, as the name suggests, simply
 * forwards in real-time the media sent by a publisher via RTP (plain
 * or encrypted) to a remote backend. Notice that, although we're using
 * the term "RTP forwarder", this feature can be used to forward data
 * channel messages as well.
 *
 * You can add a new RTP forwarder for an existing publisher using the
 * \c rtp_forward request, which has to be formatted as follows:
 *
\verbatim
{
	"request" : "rtp_forward",
	"room" : <unique numeric ID of the room the publisher is in>,
	"publisher_id" : <unique numeric ID of the publisher to relay externally>,
	"host" : "<host address to forward the RTP and data packets to>",
	"host_family" : "<ipv4|ipv6, if we need to resolve the host address to an IP; by default, whatever we get>",
	"streams" : [
		{
			"mid" : "<mid of publisher stream to forward>",
			"host" : "<host address to forward the packets to; optional, will use global one if missing>",
			"host_family" : "<optional, will use global one if missing>",
			"port" : <port to forward the packets to>,
			"ssrc" : <SSRC to use to use when forwarding; optional, and only for RTP streams, not data>,
			"pt" : <payload type to use when forwarding; optional, and only for RTP streams, not data>,
			"rtcp_port" : <port to contact to receive RTCP feedback from the recipient; optional, and only for RTP streams, not data>,
			"simulcast" : <true|false, set to true if the source is simulcast and you want the forwarder to act as a regular viewer (single stream being forwarded) or false otherwise (substreams forwarded separately); optional, default=false>,
			"port_2" : <if video and simulcasting, port to forward the packets from the second substream/layer to>,
			"ssrc_2" : <if video and simulcasting, SSRC to use to use the second substream/layer; optional>,
			"pt_2" : <if video and simulcasting, payload type to use the second substream/layer; optional>,
			"port_3" : <if video and simulcasting, port to forward the packets from the third substream/layer to>,
			"ssrc_3" : <if video and simulcasting, SSRC to use to use the third substream/layer; optional>,
			"pt_3" : <if video and simulcasting, payload type to use the third substream/layer; optional>,
		},
		{
			.. other streams, if needed..
		}
	],
	"srtp_suite" : <length of authentication tag (32 or 80); optional>,
	"srtp_crypto" : "<key to use as crypto (base64 encoded key as in SDES); optional>"
}
\endverbatim
 *
 * As you can see, you basically configure each stream to forward in a
 * dedicated object of the \c streams array: for RTP streams (audio, video)
 * this includes optionally overriding payload type or SSRC; simulcast
 * streams can be forwarded separately for each layer. The only parameters
 * you MUST specify are the host and port to send the packets to: the host
 * part can be put in the global part of the request, if all streams will
 * be sent to the same IP address, while the port must be specific to the
 * stream itself.
 *
 * Notice that, as explained above, in case you configured an \c admin_key
 * property and extended it to RTP forwarding as well, you'll need to provide
 * it in the request as well or it will be rejected as unauthorized. By
 * default no limitation is posed on \c rtp_forward .
 *
 * It's worth spending some more words on how to forward simulcast publishers,
 * as this can lead to some confusion. There are basically two ways to forward
 * a simulcast publisher:
 *
 * -# you treat the forwarder as a regular viewer, which means you still only
 * forward a single stream to the recipient, that is the highest quality
 * available at any given time: you can do that by setting
 * <code>simulcast: true</code> in the \c rtp_forward request;
 * -# you forward each substream separately instead, to different target
 * ports: you do that by specifying \c video_port_2 , \c video_port_3 and
 * optionally the other related \c _2 and \c _3 properties; this is what
 * you should use when you want to forward to a simulcast-aware Streaming
 * mountpoint (see the \ref streaming for more details).
 *
 * The two approaches are mutually exclusive: you can NOT use them together
 * in the same RTP forwarder.
 *
 * A successful request will result in an \c rtp_forward response, containing
 * the relevant info associated to the new forwarder(s):
 *
\verbatim
{
	"videoroom" : "rtp_forward",
	"room" : <unique numeric ID, same as request>,
	"publisher_id" : <unique numeric ID, same as request>,
	"forwarders" : [
		{
			"stream_id" : <unique numeric ID assigned to this forwarder, if any>,
			"type" : "<audio|video|data>",
			"host" : "<host this forwarder is streaming to, same as request if not resolved>",
			"port" : <port this forwarder is streaming to, same as request if configured>,
			"local_rtcp_port" : <local port this forwarder is using to get RTCP feedback, if any>,
			"remote_rtcp_port" : <remote port this forwarder is getting RTCP feedback from, if any>,
			"ssrc" : <SSRC this forwarder is using, same as request if configured>,
			"pt" : <payload type this forwarder is using, same as request if configured>,
			"substream" : <video substream this video forwarder is relaying, if any>,
			"srtp" : <true|false, whether the RTP stream is encrypted (not used for data)>
		},
		// Other forwarders, if configured
	]
}
\endverbatim
 *
 * To stop a previously created RTP forwarder and stop it, you can use
 * the \c stop_rtp_forward request, which has to be formatted as follows:
 *
\verbatim
{
	"request" : "stop_rtp_forward",
	"room" : <unique numeric ID of the room the publisher is in>,
	"publisher_id" : <unique numeric ID of the publisher to update>,
	"stream_id" : <unique numeric ID of the RTP forwarder>
}
\endverbatim
 *
 * A successful request will result in a \c stop_rtp_forward response:
 *
\verbatim
{
	"videoroom" : "stop_rtp_forward",
	"room" : <unique numeric ID, same as request>,
	"publisher_id" : <unique numeric ID, same as request>,
	"stream_id" : <unique numeric ID, same as request>
}
\endverbatim
 *
 * To get a list of all the forwarders in a specific room, instead, you
 * can make use of the \c listforwarders request, which has to be
 * formatted as follows:
 *
\verbatim
{
	"request" : "listforwarders",
	"room" : <unique numeric ID of the room>,
	"secret" : "<room secret; mandatory if configured>"
}
\endverbatim
 *
 * A successful request will produce a list of RTP forwarders in a
 * \c forwarders response:
 *
\verbatim
{
	"videoroom" : "forwarders",
	"room" : <unique numeric ID of the room>,
	"publishers" : [		// Array of publishers with RTP forwarders
		{	// Publisher #1
			"publisher_id" : <unique numeric ID of publisher #1>,
			"forwarders" : [		// Array of RTP forwarders
				{	// RTP forwarder #1
					"stream_id" : <unique numeric ID assigned to this RTP forwarder, if any>,
					"type" : "<audio|video|data>",
					"host" : "<host this forwarder is streaming to>",
					"port" : <port this forwarder is streaming to>,
					"local_rtcp_port" : <local port this forwarder is using to get RTCP feedback, if any>,
					"remote_rtcp_port" : <remote port this forwarder getting RTCP feedback from, if any>,
					"ssrc" : <SSRC this forwarder is using, if any>,
					"pt" : <payload type this forwarder is using, if any>,
					"substream" : <video substream this video forwarder is relaying, if any>,
					"srtp" : <true|false, whether the RTP stream is encrypted>
				},
				// Other forwarders for this publisher
			],
		},
		// Other publishers
	]
}
\endverbatim *
 *
 * To enable or disable recording on all participants while the conference
 * is in progress, you can make use of the \c enable_recording request,
 * which has to be formatted as follows:
 *
\verbatim
{
	"request" : "enable_recording",
	"room" : <unique numeric ID of the room>,
	"secret" : "<room secret; mandatory if configured>"
	"record" : <true|false, whether participants in this room should be automatically recorded or not>,
}
\endverbatim *
 *
 * Notice that, as we'll see later, participants can normally change their
 * own recording state via \c configure requests as well: this was done to
 * allow the maximum flexibility, where rather than globally or automatically
 * record something, you may want to individually record some streams and
 * to a specific file. That said, if you'd rather ensure that participants
 * can't stop their recording if a global recording is enabled, or start
 * it when the room is not supposed to be recorded instead, then you should
 * make sure the room is created with the \c lock_record property set to
 * \c true : this way, the recording state can only be changed if the room
 * secret is provided, thus ensuring that only an administrator will normally
 * be able to do that (e.g., using the \c enable_recording just introduced).
 *
 * To conclude, you can leave a room you previously joined as publisher
 * using the \c leave request. This will also implicitly unpublish you
 * if you were an active publisher in the room. The \c leave request
 * looks like follows:
 *
\verbatim
{
	"request" : "leave"
}
\endverbatim
 *
 * If successful, the response will look like this:
 *
\verbatim
{
	"videoroom" : "event",
	"leaving" : "ok"
}
\endverbatim
 *
 * Other participants will receive a "leaving" event to notify them the
 * circumstance:
 *
\verbatim
{
	"videoroom" : "event",
	"room" : <room ID>,
	"leaving : <unique ID of the participant who left>
}
\endverbatim
 *
 * If you were an active publisher, other users will also receive the
 * corresponding "unpublished" event to notify them the stream is not longer
 * available, as explained above. If you were simply lurking and not
 * publishing, the other participants will only receive the "leaving" event.
 *
 * \subsection vroomsub VideoRoom Subscribers
 *
 * In a VideoRoom, subscribers are NOT participants, but simply handles
 * that will be used exclusively to receive media from one or more publishers
 * in the room. Since they're not participants per se, they're basically
 * streams that can be (and typically are) associated to publisher handles
 * as the ones we introduced in the previous section, whether active or not.
 * In fact, the typical use case is publishers being notified about new
 * participants becoming active in the room, and as a result new subscriber
 * sessions being created to receive their media streams; as soon as the
 * publisher goes away, other participants are notified so that the related
 * subscriber handles can be removed/updated accordingly as well. As such,
 * these subscriber sessions are dependent on feedback obtained by
 * publishers, and can't exist on their own, unless you feed them the
 * right info out of band (which is impossible in rooms configured with
 * \c require_pvtid).
 *
 * To specify that a handle will be associated with a subscriber, you must use
 * the \c join request with \c ptype set to \c subscriber and specify which
 * feed to subscribe to. The exact syntax of the request is the following:
 *
\verbatim
{
	"request" : "join",
	"ptype" : "subscriber",
	"room" : <unique ID of the room to subscribe in>,
	"private_id" : <unique ID of the publisher that originated this request; optional, unless mandated by the room configuration>,
	"streams" : [
		{
			"feed" : <unique ID of publisher owning the stream to subscribe to>,
			"mid" : "<unique mid of the publisher stream to subscribe to; optional>"
			"crossrefid" : "<id to map this subscription with entries in streams list; optional>"
			// Optionally, simulcast or SVC targets (defaults if missing)
		},
		// Other streams to subscribe to
	]
}
\endverbatim
 *
 * As you can see, it's just a matter of specifying the list of streams to
 * subscribe to: in particular, you have to provide an array of objects,
 * where each objects represents a specific stream (or group of streams)
 * you're interested in. For each object, the \c feed_id indicating the
 * publisher owning the stream(s) is mandatory, while the related \c mid
 * is optional: this gives you some flexibility when subscribing, as
 * only providing a \c feed_id will indicate you're interested in ALL
 * the stream from that publisher, while providing a \c mid as well will
 * indicate you're interested in a stream in particular. Since you can
 * provide an array of streams, just specifying the \c feed_id or explicitly
 * listing all the \c feed_id + \c mid combinations is equivalent: of
 * course, different objects in the array can indicate different publishers,
 * allowing you to combine streams from different sources in the same subscription.
 * Notice that if a publisher stream is marked as \c disabled and you try
 * to subscribe to it, it will be skipped silently.
 *
 * Depending on whether the subscription will refer to a
 * single publisher (legacy approach) or to streams coming from different
 * publishers (multistream), the list of streams may differ. The ability
 * to single out the streams to subscribe to is particularly useful in
 * case you don't want to, or can't, subscribe to all available media:
 * e.g., you know a publisher is sending both audio and video, but video
 * is in a codec you don't support or you don't have bandwidth for both;
 * or maybe there are 10 participants in the room, but you only want video
 * from the 3 most active speakers; and so on. The content of the \c streams
 * array will shape what the SDP offer the plugin will send will look like,
 * so that eventually a subscription for the specified streams will take place.
 * Notice that, while for backwards compatibility you can still use the
 * old \c feed, \c audio, \c video, \c data, \c offer_audio, \c offer_video and
 * \c offer_data named properties, they're now deprecated and so you're
 * highly encouraged to use this new drill-down \c streams list instead.
 *
 * As anticipated, if successful this request will generate a new JSEP SDP
 * offer, which will accompany an \c attached event:
 *
\verbatim
{
	"videoroom" : "attached",
	"room" : <room ID>,
	"streams" : [
		{
			"mindex" : <unique m-index of this stream>,
			"mid" : "<unique mid of this stream>",
			"type" : "<type of this stream's media (audio|video|data)>",
			"feed_id" : <unique ID of the publisher originating this stream>,
			"feed_mid" : "<unique mid of this publisher's stream>",
			"feed_display" : "<display name of this publisher, if any>",
			"send" : <true|false; whether we configured the stream to relay media>,
			"ready" : <true|false; whether this stream is ready to start sending media (will be false at the beginning)>
		},
		// Other streams in the subscription, if any
	]
}
\endverbatim
 *
 * As you can see, a summary of the streams we subscribed to will be sent back,
 * which will be useful on the client side for both mapping and rendering purposes.
 *
 * At this stage, to complete the setup of the PeerConnection the subscriber is
 * supposed to send a JSEP SDP answer back to the plugin. This is done
 * by means of a \c start request, which in this case MUST be associated
 * with a JSEP SDP answer but otherwise requires no arguments:
 *
\verbatim
{
	"request" : "start"
}
\endverbatim
 *
 * If successful this request returns a \c started event:
 *
\verbatim
{
	"videoroom" : "event",
	"started" : "ok"
}
\endverbatim
 *
 * Once this is done, all that's needed is waiting for the WebRTC PeerConnection
 * establishment to succeed. As soon as that happens, the VideoRoom plugin
 * can start relaying media the recipient subscribed to.
 *
 * Once a WebRTC PeerConnection has been established for a subscriber, in
 * case you want to update a subscription you have to use the \c subscribe ,
 * \c unsubscribe or \update methods: as the names of the requests suggest, the
 * former allows you to add more streams to subscribe to, the second
 * instructs the plugin to remove streams you're currently subscribe to,
 * while the latter allows you to perform both operations at the same time.
 * Any of those requests will trigger a renegotiation, if they were successful,
 * meaning the plugin will send you a new JSEP offer you'll have to reply
 * to with an answer: to send the answer, just use the same \c start request
 * we already described above. Notice that renegotiations may not be
 * triggered right away, e.g., whenever you're trying to update a session
 * and the plugin is still in the process of renegoting a previous update
 * for the same subscription: in that case, an update will be scheduled
 * and a renegotiation will be triggered as soon as it's viable, and an
 * empty \c updating event will be triggered instead to notify the caller
 * that the management of that request has been postponed. It's also
 * important to point out that the number of offers generated in response
 * to those requests may not match the amount of requests: in fact, since
 * requests are postponed, a single offer may be sent in response to
 * multiple requests to update a subscription at the same time, thus
 * addressing them all in a cumulative way. This means clients should
 * never expect an offer any time they request one.
 *
 * The syntax of the \c subscribe mirrors the one for new subscriptions,
 * meaning you use the same \c streams array to address the new streams
 * you want to receive, and formatted the same way:
 *
\verbatim
{
	"request" : "subscribe",
	"streams" : [
		{
			"feed" : <unique ID of publisher owning the new stream to subscribe to>,
			"mid" : "<unique mid of the publisher stream to subscribe to; optional>"
			"crossrefid" : "<id to map this subscription with entries in streams list; optional>"
			// Optionally, simulcast or SVC targets (defaults if missing)
		},
		// Other new streams to subscribe to
	]
}
\endverbatim
 *
 * This means the exact same considerations we made on \c streams before
 * apply here as well: whatever they represent, will indicate the willingness
 * to subscribe to the related stream. Notice that if you were already
 * subscribed to one of the new streams indicated here, you'll subscribe
 * to it again in a different m-line, so it's up to you to ensure you
 * avoid duplicates (unless that's what you wanted, e.g., for testing
 * purposes). In case the update was successful, you'll get an \c updated
 * event, containing the updated layout of all subscriptions (pre-existing
 * and new ones), and a new JSEP offer to renegotiate the session:
 *
\verbatim
{
	"videoroom" : "updated",
	"room" : <room ID>,
	"streams": [
		{
			"mindex" : <unique m-index of this stream>,
			"mid" : "<unique mid of this stream>",
			"type" : "<type of this stream's media (audio|video|data)>",
			"feed_id" : <unique ID of the publisher originating this stream>,
			"feed_mid" : "<unique mid of this publisher's stream>",
			"feed_display" : "<display name of this publisher, if any>",
			"send" : <true|false; whether we configured the stream to relay media>,
			"ready" : <true|false; whether this stream is ready to start sending media (will be false at the beginning)>
		},
		// Other streams in the subscription, if any; old and new
	]
}
\endverbatim
 *
 * Notice that if your \c subscribe request didn't change anything as far
 * as the SDP negotiation is concerned (e.g., subscribing to new data streams
 * where a datachannel existed already), you'll simply get an \c updated
 * event back with no \c streams object.
 *
 * As explained before, in case the message contains a JSEP offer (which may
 * not be the case if no change occurred), then clients will need to send
 * a new JSEP answer with a \c start request to close this renegotiation.
 *
 * The \c unsubscribe request works pretty much the same way, with the
 * difference that the \c streams array you provide to specify what to
 * unsubscribe from may look different. Specifically, the syntax looks
 * like this:
 *
\verbatim
{
	"request" : "unsubscribe",
	"streams" : [
		{
			"feed" : <unique ID of publisher owning the new stream to unsubscribe from; optional>,
			"mid" : "<unique mid of the publisher stream to unsubscribe from; optional>"
			"sub_mid" : "<unique mid of the subscriber stream to unsubscribe; optional>"
		},
		// Other streams to unsubscribe from
	]
}
\endverbatim
 *
 * This means that you have different ways to specify what to unsubscribe from:
 * if an object only specifies \c feed_id, then all the subscription streams that
 * were receiving media from that publisher will be removed; if an object
 * specifies \c feed_id and \c mid, then all the subscription streams that
 * were receiving media from the publisher stream with the related mid will be
 * removed; finally, if an object only specifies \c sub_mid instead, then
 * only the stream in the subscription that is addressed by the related mid
 * (subscription mid, no relation to the publishers') will be removed. As
 * such, you have a great deal of flexibility in how to unsubscribe from
 * media. Notice that multiple streams may be removed in case you refer
 * to the "source" ( \c feed_id ), rather than the "sink" ( \c sub_mid ),
 * especially in case the subscription contained duplicates or multiple
 * streams from the same publisher.
 *
 * A successful \c unsubscribe will result in exactly the same \c updated
 * event \c subscribe triggers, so the same considerations apply with
 * respect to the potential need of a renegotiation and how to complete
 * it with a \c start along a JSEP answer. Again, if \c unsubscribe didn't
 * result in SDP changes (e.g., unsubscribing from a data channel stream),
 * you'll simply get an \c updated event back with no \c streams object.
 *
 * As anticipated, the \c update request allows you to combine changes
 * to a subscription where you may want to both subscribe to new streams,
 * and unsubscribe from existing ones, which the existing \c subscribe
 * and \c unsubscribe requests wouldn't allow you to do as they work
 * exclusively on the action specified by their name. The syntax for
 * the \c update request is very similar to the previous method, meaning
 * arrays are still used to address the streams to work on, with the key
 * difference that they won't be named \c streams, but \c subscribe and
 * \c unsubscribe instead:
 *
\verbatim
{
	"request" : "update",
	"subscribe" : [
		{
			"feed" : <unique ID of publisher owning the new stream to subscribe to>,
			"mid" : "<unique mid of the publisher stream to subscribe to; optional>"
			"crossrefid" : "<id to map this subscription with entries in streams list; optional>"
			// Optionally, simulcast or SVC targets (defaults if missing)
		},
		// Other new streams to subscribe to
	],
	"unsubscribe" : [
		{
			"feed" : <unique ID of publisher owning the new stream to unsubscribe from; optional>,
			"mid" : "<unique mid of the publisher stream to unsubscribe from; optional>"
			"sub_mid" : "<unique mid of the subscriber stream to unsubscribe; optional>"
		},
		// Other streams to unsubscribe from
	]
}
\endverbatim
 *
 * Both the \c subscribe and \c unsubscribe arrays are optional, which means
 * that an \c update request to only subscribe to new streams will be
 * functionally equivalent to a \c subscribe request, and an \c update
 * request to only unsubscribe will be functionally equivalent to an
 * \c unsubscribe request instead. That said, one of the two must be
 * provided, which means that an \c update request that doesn't include
 * either of them will result in an error.
 *
 * A successful \c update will result in exactly the same \c updated event
 * \c subscribe and \c unsubscribe trigger, so the same considerations apply
 * with respect to the potential need of a renegotiation and how to complete
 * it with a \c start along a JSEP answer. Again, if \c update didn't
 * result in SDP changes, you'll simply get an \c updated event back with
 * no \c streams object.
 *
 * Notice that, in case you want to trigger an ICE restart rather than
 * updating a subscription, you'll have to use a different request, named
 * \c configure: this will be explained in a few paragraphs.
 *
 * As a subscriber, you can temporarily pause and resume the whole media delivery
 * with a \c pause and, again, \c start request (in this case without any JSEP
 * SDP answer attached). Neither expect other arguments, as the context
 * is implicitly derived from the handle they're sent on:
 *
\verbatim
{
	"request" : "pause"
}
\endverbatim
 *
\verbatim
{
	"request" : "start"
}
\endverbatim
 *
 * Unsurprisingly, they just result in, respectively, \c paused and
 * \c started events:
 *
\verbatim
{
	"videoroom" : "event",
	"paused" : "ok"
}
\endverbatim
 *
\verbatim
{
	"videoroom" : "event",
	"started" : "ok"
}
\endverbatim
 *
 * For more drill-down manipulations of a subscription, a \c configure
 * request can be used instead. This request allows subscribers to dynamically
 * change some properties associated to their media subscription, e.g.,
 * in terms of what should and should not be sent at a specific time. A
 * \c configure request must be formatted as follows:
 *
\verbatim
{
	"request" : "configure",
	"streams" : [
		{
			"mid" : <mid of the m-line to refer to>,
			"send" : <true|false, depending on whether the mindex media should be relayed or not; optional>,
			"substream" : <substream to receive (0-2), in case simulcasting is enabled; optional>,
			"temporal" : <temporal layers to receive (0-2), in case simulcasting is enabled; optional>,
			"fallback" : <How much time (in us, default 250000) without receiving packets will make us drop to the substream below; optional>,
			"spatial_layer" : <spatial layer to receive (0-2), in case VP9-SVC is enabled; optional>,
			"temporal_layer" : <temporal layers to receive (0-2), in case VP9-SVC is enabled; optional>,
			"audio_level_average" : "<if provided, overrides the room audio_level_average for this user; optional>",
			"audio_active_packets" : "<if provided, overrides the room audio_active_packets for this user; optional>",
			"min_delay" : <minimum delay to enforce via the playout-delay RTP extension, in blocks of 10ms; optional>,
			"max_delay" : <maximum delay to enforce via the playout-delay RTP extension, in blocks of 10ms; optional>,
		},
		// Other streams, if any
	],
	"restart" : <trigger an ICE restart; optional>
}
\endverbatim
 *
 * As you can see, the \c mid and \c send properties can be used as a media-level
 * pause/resume functionality ("only mute/unmute this mid"), whereas \c pause
 * and \c start simply pause and resume all streams at the same time.
 * The \c substream and \c temporal properties, instead, only make sense
 * when the publisher is configured with video simulcasting support, and
 * as such the subscriber is interested in receiving a specific substream
 * or temporal layer, rather than any other of the available ones: notice
 * that for them to work you'll have to specify the \c mid as well, as the same
 * subscription may be receiving simulcast stream from multiple publishers.
 * The \c spatial_layer and \c temporal_layer have exactly the same meaning,
 * but within the context of VP9-SVC publishers, and will have no effect
 * on subscriptions associated to regular publishers.
 *
 * As anticipated, \c configure is also the request you use when you want
 * to trigger an ICE restart for a subscriber: in fact, while publishers
 * can force a restart themselves by providing the right JSEP offer, subscribers
 * always receive an offer from Janus instead, and as such have to
 * explicitly ask for a dedicated offer when an ICE restart is needed;
 * in that case, just set \c restart to \c true in a \c configure request,
 * and a new JSEP offer with ICE restart information will be sent to the
 * client, to which the client will have to reply, as usual, via \c start
 * along a JSEP answer. This documentation doesn't explain when or why
 * an ICE restart is needed or appropriate: please refer to the ICE RFC
 * or other sources of information for that.
 *
 * Another interesting feature that subscribers can take advantage of is the
 * so-called publisher "switching". Basically, when subscribed to one or more
 * publishers and receiving media from them, you can at any time "switch"
 * any of the subscription streams to a different publisher, and as such
 * start receiving media on the related m-line from that publisher instead,
 * all without doing a new \c subscribe or \c unsubscribe, and so without
 * the need of doing any renegotiation at all; just some logic changes.
 * Think of it as changing channel on a TV: you keep on using the same
 * PeerConnection, the plugin simply changes the source of the media
 * transparently. Of course, while powerful and effective this request has
 * some limitations: in fact, the source (audio or video) that you switch
 * to must have the same media configuration (e.g., same codec) as the source
 * you're replacing. In fact, since the same PeerConnection is used for this
 * feature and no renegotiation is taking place, switching to a stream with
 * a different configuration would result in media incompatible with the
 * PeerConnection setup being relayed to the subscriber (e.g., negotiated
 * VP9, but new source is H.264), and as such in no audio/video being played;
 * in that case, you'll need a \c subscribe instead, and a new m-line.
 *
 * That said, a \c switch request must be formatted like this:
 *
\verbatim
{
	"request" : "switch",
	"streams" : [
		{
			"feed" : <unique ID of the publisher the new source is from>,
			"mid" : "<unique mid of the source we want to switch to>",
			"sub_mid" : "<unique mid of the stream we want to pipe the new source to>"
		},
		{
			// Other updates, if any
		}
	]
}
\endverbatim
 *
 * While apparently convoluted, this is actually a quite effective and powerful
 * way of updating subscriptions without renegotiating. In fact, it allows for
 * full or partial switches: for instance, sometimes you may want to replace all
 * audio and video streams (e.g., switching from Bob to Alice in a "legacy"
 * VideoRoom usage, where each PeerConnection subscription is a different
 * publisher), or just replace a subset of them (e.g., you have a subscription
 * with three video slots, and you change one of them depending on the loudest
 * speaker). What to replace is dictated by the \c streams array, where each
 * object in the array contains all the info needed for the switch to take
 * place: in particular, you must specify which of your subscription m-lines
 * you're going to update, via \c sub_mid , and which publisher stream should
 * now start to feed it via \c feed and \c mid.
 *
 * If successful, the specified subscriptions will be updated, meaning they'll
 * be unsubscribed from the previous publisher stream, and subscribed to the
 * new publisher stream instead, all without a renegotiation (so no new SDP
 * offer/answer exchange to take care of). The event to confirm the switch
 * was successful will look like this:
 *
\verbatim
{
	"videoroom" : "event",
	"switched" : "ok",
	"room" : <room ID>,
	"changes" : <number of successful changes (may be smaller than the size of the streams array provided in the request)>,
	"streams" : [
		// Current configuration of the subscription, same format as when subscribing
		// Will contain info on all streams, not only those that have been updated
	]
}
\endverbatim
 *
 * Notice that, while a \c switch request usually doesn't require a renegotiation,
 * it \b MIGHT trigger one nevertheless: in fact, if a "switch" request assigns
 * a new publisher stream to a previously inactive subscriber stream, then
 * a renegotiation to re-activate that stream will be needed as well, as
 * otherwise the packets from the new source will not be relayed.
 *
 * Finally, to close a subscription and tear down the related PeerConnection,
 * you can use the \c leave request. Since context is implicit, no other
 * argument is required:
 *
\verbatim
{
	"request" : "leave"
}
\endverbatim
 *
 * If successful, the plugin will attempt to tear down the PeerConnection,
 * and will send back a \c left event:
 *
\verbatim
{
	"videoroom" : "event",
	"left" : "ok",
}
\endverbatim
 */

#include "plugin.h"

#include <jansson.h>
#include <netdb.h>
#include <math.h>

#include "../debug.h"
#include "../apierror.h"
#include "../config.h"
#include "../mutex.h"
#include "../rtp.h"
#include "../rtpsrtp.h"
#include "../rtcp.h"
#include "../record.h"
#include "../sdp-utils.h"
#include "../utils.h"
#include "../ip-utils.h"
#include <sys/types.h>
#include <sys/socket.h>

#include "speech_mix.h"
#include <sys/time.h>


/* Plugin information */
#define JANUS_VIDEOROOM_VERSION			9
#define JANUS_VIDEOROOM_VERSION_STRING	"0.0.9"
#define JANUS_VIDEOROOM_DESCRIPTION		"This is a plugin implementing a videoconferencing SFU (Selective Forwarding Unit) for Janus, that is an audio/video router."
#define JANUS_VIDEOROOM_NAME			"JANUS VideoRoom plugin"
#define JANUS_VIDEOROOM_AUTHOR			"Meetecho s.r.l."
#define JANUS_VIDEOROOM_PACKAGE			"janus.plugin.videoroom"

/* Plugin methods */
janus_plugin *create(void);
int janus_videoroom_init(janus_callbacks *callback, const char *config_path);
void janus_videoroom_destroy(void);
int janus_videoroom_get_api_compatibility(void);
int janus_videoroom_get_version(void);
const char *janus_videoroom_get_version_string(void);
const char *janus_videoroom_get_description(void);
const char *janus_videoroom_get_name(void);
const char *janus_videoroom_get_author(void);
const char *janus_videoroom_get_package(void);
void janus_videoroom_create_session(janus_plugin_session *handle, int *error);
struct janus_plugin_result *janus_videoroom_handle_message(janus_plugin_session *handle, char *transaction, json_t *message, json_t *jsep);
json_t *janus_videoroom_handle_admin_message(json_t *message);
void janus_videoroom_setup_media(janus_plugin_session *handle);
void janus_videoroom_incoming_rtp(janus_plugin_session *handle, janus_plugin_rtp *packet);
void janus_videoroom_incoming_rtcp(janus_plugin_session *handle, janus_plugin_rtcp *packet);
void janus_videoroom_incoming_data(janus_plugin_session *handle, janus_plugin_data *packet);
void janus_videoroom_data_ready(janus_plugin_session *handle);
void janus_videoroom_slow_link(janus_plugin_session *handle, int mindex, gboolean video, gboolean uplink);
void janus_videoroom_hangup_media(janus_plugin_session *handle);
void janus_videoroom_destroy_session(janus_plugin_session *handle, int *error);
json_t *janus_videoroom_query_session(janus_plugin_session *handle);
void janus_videoroom_outgoing_congestion(janus_plugin_session *handle, int congestion_flag);
int janus_videoroom_query_user_name(janus_plugin_session *handle, char * name, int name_size);


/* Plugin setup */
static janus_plugin janus_videoroom_plugin =
	JANUS_PLUGIN_INIT (
		.init = janus_videoroom_init,
		.destroy = janus_videoroom_destroy,

		.get_api_compatibility = janus_videoroom_get_api_compatibility,
		.get_version = janus_videoroom_get_version,
		.get_version_string = janus_videoroom_get_version_string,
		.get_description = janus_videoroom_get_description,
		.get_name = janus_videoroom_get_name,
		.get_author = janus_videoroom_get_author,
		.get_package = janus_videoroom_get_package,

		.create_session = janus_videoroom_create_session,
		.handle_message = janus_videoroom_handle_message,
		.handle_admin_message = janus_videoroom_handle_admin_message,
		.setup_media = janus_videoroom_setup_media,
		.incoming_rtp = janus_videoroom_incoming_rtp,
		.incoming_rtcp = janus_videoroom_incoming_rtcp,
		.incoming_data = janus_videoroom_incoming_data,
		.data_ready = janus_videoroom_data_ready,
		.slow_link = janus_videoroom_slow_link,
		.hangup_media = janus_videoroom_hangup_media,
		.destroy_session = janus_videoroom_destroy_session,
		.query_session = janus_videoroom_query_session,
		.outgoing_congestion = janus_videoroom_outgoing_congestion,
		.query_user_name = janus_videoroom_query_user_name
	);

/* Plugin creator */
janus_plugin *create(void) {
	JANUS_LOG(LOG_VERB, "%s created!\n", JANUS_VIDEOROOM_NAME);
	return &janus_videoroom_plugin;
}

/* Parameter validation */
static struct janus_json_parameter request_parameters[] = {
	{"request", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter adminkey_parameters[] = {
	{"admin_key", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter create_parameters[] = {
	{"description", JSON_STRING, 0},
	{"is_private", JANUS_JSON_BOOL, 0},
	{"allowed", JSON_ARRAY, 0},
	{"secret", JSON_STRING, 0},
	{"pin", JSON_STRING, 0},
	{"require_pvtid", JANUS_JSON_BOOL, 0},
	{"signed_tokens", JANUS_JSON_BOOL, 0},
	{"bitrate", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"bitrate_cap", JANUS_JSON_BOOL, 0},
	{"fir_freq", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"publishers", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audiocodec", JSON_STRING, 0},
	{"videocodec", JSON_STRING, 0},
	{"vp9_profile", JSON_STRING, 0},
	{"h264_profile", JSON_STRING, 0},
	{"opus_fec", JANUS_JSON_BOOL, 0},
	{"opus_dtx", JANUS_JSON_BOOL, 0},
	{"video_svc", JANUS_JSON_BOOL, 0},
	{"audiolevel_ext", JANUS_JSON_BOOL, 0},
	{"audiolevel_event", JANUS_JSON_BOOL, 0},
	{"audio_active_packets", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_level_average", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"videoorient_ext", JANUS_JSON_BOOL, 0},
	{"playoutdelay_ext", JANUS_JSON_BOOL, 0},
	{"transport_wide_cc_ext", JANUS_JSON_BOOL, 0},
	{"record", JANUS_JSON_BOOL, 0},
	{"rec_dir", JSON_STRING, 0},
	{"lock_record", JANUS_JSON_BOOL, 0},
	{"permanent", JANUS_JSON_BOOL, 0},
	{"notify_joining", JANUS_JSON_BOOL, 0},
	{"require_e2ee", JANUS_JSON_BOOL, 0},
	{"dummy_publisher", JANUS_JSON_BOOL, 0},
	{"dummy_streams", JANUS_JSON_ARRAY, 0}
};
static struct janus_json_parameter edit_parameters[] = {
	{"secret", JSON_STRING, 0},
	{"new_description", JSON_STRING, 0},
	{"new_is_private", JANUS_JSON_BOOL, 0},
	{"new_secret", JSON_STRING, 0},
	{"new_pin", JSON_STRING, 0},
	{"new_require_pvtid", JANUS_JSON_BOOL, 0},
	{"new_bitrate", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"new_fir_freq", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"new_publishers", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"new_lock_record", JANUS_JSON_BOOL, 0},
	{"new_rec_dir", JSON_STRING, 0},
	{"permanent", JANUS_JSON_BOOL, 0}
};
static struct janus_json_parameter room_parameters[] = {
	{"room", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter roomopt_parameters[] = {
	{"room", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter roomstr_parameters[] = {
	{"room", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter roomstropt_parameters[] = {
	{"room", JSON_STRING, 0}
};
static struct janus_json_parameter id_parameters[] = {
	{"id", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter idopt_parameters[] = {
	{"id", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter idstr_parameters[] = {
	{"id", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter idstropt_parameters[] = {
	{"id", JSON_STRING, 0}
};
static struct janus_json_parameter pid_parameters[] = {
	{"publisher_id", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter pidstr_parameters[] = {
	{"publisher_id", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter feed_parameters[] = {
	{"feed", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter feedstr_parameters[] = {
	{"feed", JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter feedopt_parameters[] = {
	{"feed", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter feedstropt_parameters[] = {
	{"feed", JSON_STRING, 0}
};
static struct janus_json_parameter destroy_parameters[] = {
	{"permanent", JANUS_JSON_BOOL, 0}
};
static struct janus_json_parameter allowed_parameters[] = {
	{"secret", JSON_STRING, 0},
	{"action", JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"allowed", JSON_ARRAY, 0}
};
static struct janus_json_parameter kick_parameters[] = {
	{"secret", JSON_STRING, 0}
};
static struct janus_json_parameter moderate_parameters[] = {
	{"secret", JSON_STRING, 0},
	{"mid", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"mute", JANUS_JSON_BOOL, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter join_parameters[] = {
	{"ptype", JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"audio", JANUS_JSON_BOOL, 0},
	{"video", JANUS_JSON_BOOL, 0},
	{"data", JANUS_JSON_BOOL, 0},
	{"bitrate", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"record", JANUS_JSON_BOOL, 0},
	{"filename", JSON_STRING, 0},
	{"token", JSON_STRING, 0}
};
static struct janus_json_parameter publish_parameters[] = {
	{"descriptions", JANUS_JSON_ARRAY, 0},
	{"audiocodec", JSON_STRING, 0},
	{"videocodec", JSON_STRING, 0},
	{"bitrate", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"keyframe", JANUS_JSON_BOOL, 0},
	{"record", JANUS_JSON_BOOL, 0},
	{"filename", JSON_STRING, 0},
	{"display", JSON_STRING, 0},
	{"secret", JSON_STRING, 0},
	{"audio_level_averge", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_active_packets", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* Deprecated, use mid+send instead */
	{"audio", JANUS_JSON_BOOL, 0},	/* Deprecated! */
	{"video", JANUS_JSON_BOOL, 0},	/* Deprecated! */
	{"data", JANUS_JSON_BOOL, 0},	/* Deprecated! */
	/* The following are just to force a renegotiation and/or an ICE restart */
	{"update", JANUS_JSON_BOOL, 0},
	{"restart", JANUS_JSON_BOOL, 0}
};
static struct janus_json_parameter publish_stream_parameters[] = {
	{"mid", JANUS_JSON_STRING, 0},
	{"send", JANUS_JSON_BOOL, 0},
	/* For the playout-delay RTP extension, if negotiated */
	{"min_delay", JSON_INTEGER, 0},
	{"max_delay", JSON_INTEGER, 0},
};
static struct janus_json_parameter publish_desc_parameters[] = {
	{"mid", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"description", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter record_parameters[] = {
	{"record", JANUS_JSON_BOOL, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter rtp_forward_parameters[] = {
	{"host", JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"host_family", JSON_STRING, 0},
	{"simulcast", JANUS_JSON_BOOL, 0},
	{"srtp_suite", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"srtp_crypto", JSON_STRING, 0},
	{"streams", JANUS_JSON_ARRAY, 0},
	/* Deprecated parameters, use the streams array instead */
	{"video_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_rtcp_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_ssrc", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_pt", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_port_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_ssrc_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_pt_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_port_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_ssrc_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"video_pt_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_rtcp_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_ssrc", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_pt", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"data_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
};
static struct janus_json_parameter rtp_forward_stream_parameters[] = {
	{"mid", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"host", JSON_STRING, 0},
	{"host_family", JSON_STRING, 0},
	{"port", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE},
	{"rtcp_port", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"ssrc", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"pt", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"simulcast", JANUS_JSON_BOOL, 0},
	{"srtp_suite", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"srtp_crypto", JSON_STRING, 0},
	{"port_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"ssrc_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"pt_2", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"port_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"ssrc_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"pt_3", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter stop_rtp_forward_parameters[] = {
	{"stream_id", JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE}
};
static struct janus_json_parameter publisher_parameters[] = {
	{"display", JSON_STRING, 0}
};
static struct janus_json_parameter configure_stream_parameters[] = {
	{"mid", JANUS_JSON_STRING, 0},
	{"send", JANUS_JSON_BOOL, 0},
	/* For talk detection */
	{"audio_level_averge", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"audio_active_packets", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For VP8 (or H.264) simulcast */
	{"substream", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"fallback", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For VP9 SVC */
	{"spatial_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For the playout-delay RTP extension, if negotiated */
	{"min_delay", JSON_INTEGER, 0},
	{"max_delay", JSON_INTEGER, 0},
};
static struct janus_json_parameter configure_parameters[] = {
	{"streams", JANUS_JSON_ARRAY, 0},
	/* The following is to handle a renegotiation */
	{"update", JANUS_JSON_BOOL, 0},
	/* The following is to force a restart */
	{"restart", JANUS_JSON_BOOL, 0},
	/* Deprecated properties, use mid+send instead */
	{"audio", JANUS_JSON_BOOL, 0},	/* Deprecated */
	{"video", JANUS_JSON_BOOL, 0},	/* Deprecated */
	{"data", JANUS_JSON_BOOL, 0}	/* Deprecated */
};
static struct janus_json_parameter subscriber_parameters[] = {
	{"streams", JANUS_JSON_ARRAY, 0},
	{"private_id", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"close_pc", JANUS_JSON_BOOL, 0},
	/* All the following parameters are deprecated: use streams instead */
	{"audio", JANUS_JSON_BOOL, 0},
	{"video", JANUS_JSON_BOOL, 0},
	{"data", JANUS_JSON_BOOL, 0},
	{"offer_audio", JANUS_JSON_BOOL, 0},
	{"offer_video", JANUS_JSON_BOOL, 0},
	{"offer_data", JANUS_JSON_BOOL, 0},
	/* For VP8 (or H.264) simulcast */
	{"substream", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"fallback", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For VP9 SVC */
	{"spatial_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
};
static struct janus_json_parameter subscriber_stream_parameters[] = {
	{"mid", JANUS_JSON_STRING, 0},
	{"crossrefid", JANUS_JSON_STRING, 0},
	/* For VP8 (or H.264) simulcast */
	{"substream", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"substream", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For VP9 SVC */
	{"spatial_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal_layer", JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"spatial_layer", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"temporal_layer", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	/* For the playout-delay RTP extension, if negotiated */
	{"min_delay", JSON_INTEGER, 0},
	{"max_delay", JSON_INTEGER, 0}
};
static struct janus_json_parameter subscriber_update_parameters[] = {
	{"streams", JANUS_JSON_ARRAY, JANUS_JSON_PARAM_REQUIRED}
};
static struct janus_json_parameter subscriber_combined_update_parameters[] = {
	{"subscribe", JANUS_JSON_ARRAY, 0},
	{"unsubscribe", JANUS_JSON_ARRAY, 0}
};
static struct janus_json_parameter subscriber_remove_parameters[] = {
	//~ {"feed", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_POSITIVE},
	{"mid", JANUS_JSON_STRING, 0},
	{"sub_mid", JANUS_JSON_STRING, 0}
};
static struct janus_json_parameter switch_parameters[] = {
	{"streams", JANUS_JSON_ARRAY, 0}
};
static struct janus_json_parameter switch_update_parameters[] = {
	//~ {"feed", JANUS_JSON_INTEGER, JANUS_JSON_PARAM_REQUIRED | JANUS_JSON_PARAM_POSITIVE},
	{"mid", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED},
	{"sub_mid", JANUS_JSON_STRING, JANUS_JSON_PARAM_REQUIRED}
};

/* Static configuration instance */
static janus_config *config = NULL;
static const char *config_folder = NULL;
static janus_mutex config_mutex = JANUS_MUTEX_INITIALIZER;

/* Useful stuff */
static volatile gint initialized = 0, stopping = 0;
static gboolean notify_events = TRUE;
static gboolean string_ids = FALSE;
static gboolean ipv6_disabled = FALSE;
static janus_callbacks *gateway = NULL;
static GThread *handler_thread;
static void *janus_videoroom_handler(void *data);
static void janus_videoroom_relay_rtp_packet(gpointer data, gpointer user_data);
static void janus_videoroom_relay_rtp_packet_ex(gpointer data, gpointer user_data, gboolean mixed_audio_flag);
static void janus_videoroom_relay_data_packet(gpointer data, gpointer user_data);
static void janus_videoroom_hangup_media_internal(gpointer session_data);

typedef enum janus_videoroom_p_type {
	janus_videoroom_p_type_none = 0,
	janus_videoroom_p_type_subscriber,			/* Generic subscriber */
	janus_videoroom_p_type_publisher,			/* Participant (for receiving events) and optionally publisher */
} janus_videoroom_p_type;

typedef enum janus_videoroom_media {
	JANUS_VIDEOROOM_MEDIA_NONE = 0,
	JANUS_VIDEOROOM_MEDIA_AUDIO,
	JANUS_VIDEOROOM_MEDIA_VIDEO,
	JANUS_VIDEOROOM_MEDIA_DATA
} janus_videoroom_media;
static const char *janus_videoroom_media_str(janus_videoroom_media type) {
	switch(type) {
		case JANUS_VIDEOROOM_MEDIA_AUDIO: return "audio";
		case JANUS_VIDEOROOM_MEDIA_VIDEO: return "video";
		case JANUS_VIDEOROOM_MEDIA_DATA: return "data";
		case JANUS_VIDEOROOM_MEDIA_NONE:
		default:
			break;
	}
	return NULL;
}
static janus_sdp_mtype janus_videoroom_media_sdptype(janus_videoroom_media type) {
	switch(type) {
		case JANUS_VIDEOROOM_MEDIA_AUDIO: return JANUS_SDP_AUDIO;
		case JANUS_VIDEOROOM_MEDIA_VIDEO: return JANUS_SDP_VIDEO;
		case JANUS_VIDEOROOM_MEDIA_DATA: return JANUS_SDP_APPLICATION;
		case JANUS_VIDEOROOM_MEDIA_NONE:
		default:
			break;
	}
	return JANUS_SDP_OTHER;
}

typedef struct janus_videoroom_message {
	janus_plugin_session *handle;
	char *transaction;
	json_t *message;
	json_t *jsep;
} janus_videoroom_message;
static GAsyncQueue *messages = NULL;
static janus_videoroom_message exit_message;


typedef struct janus_videoroom {
	guint64 room_id;			/* Unique room ID (when using integers) */
	gchar *room_id_str;			/* Unique room ID (when using strings) */
	gchar *room_name;			/* Room description */
	gchar *room_secret;			/* Secret needed to manipulate (e.g., destroy) this room */
	gchar *room_pin;			/* Password needed to join this room, if any */
	gboolean is_private;		/* Whether this room is 'private' (as in hidden) or not */
	gboolean require_pvtid;		/* Whether subscriptions in this room require a private_id */
	gboolean signed_tokens;		/* Whether signed tokens are required (assuming they're enabled in the core)  */
	gboolean require_e2ee;		/* Whether end-to-end encrypted publishers are required */
	gboolean dummy_publisher;	/* Whether this room has a dummy publisher to use for placeholder subscriptions */
	int max_publishers;			/* Maximum number of concurrent publishers */
	uint32_t bitrate;			/* Global bitrate limit */
	gboolean bitrate_cap;		/* Whether the above limit is insormountable */
	uint16_t fir_freq;			/* Regular FIR frequency (0=disabled) */
	janus_audiocodec acodec[5];	/* Audio codec(s) to force on publishers */
	janus_videocodec vcodec[5];	/* Video codec(s) to force on publishers */
	char *vp9_profile;			/* VP9 codec profile to prefer, if more are negotiated */
	char *h264_profile;			/* H.264 codec profile to prefer, if more are negotiated */
	gboolean do_opusfec;		/* Whether inband FEC must be negotiated (note: only available for Opus) */
	gboolean do_opusdtx;		/* Whether DTX must be negotiated (note: only available for Opus) */
	gboolean do_svc;			/* Whether SVC must be done for video (note: only available for VP9 right now) */
	gboolean audiolevel_ext;	/* Whether the ssrc-audio-level extension must be negotiated or not for new publishers */
	gboolean audiolevel_event;	/* Whether to emit event to other users about audiolevel */
	int audio_active_packets;	/* Amount of packets with audio level for checkup */
	int audio_level_average;	/* Average audio level */
	gboolean videoorient_ext;	/* Whether the video-orientation extension must be negotiated or not for new publishers */
	gboolean playoutdelay_ext;	/* Whether the playout-delay extension must be negotiated or not for new publishers */
	gboolean transport_wide_cc_ext;	/* Whether the transport wide cc extension must be negotiated or not for new publishers */
	gboolean record;			/* Whether the feeds from publishers in this room should be recorded */
	char *rec_dir;				/* Where to save the recordings of this room, if enabled */
	gboolean lock_record;		/* Whether recording state can only be changed providing the room secret */
	GHashTable *participants;	/* Map of potential publishers (we get subscribers from them) */
	GHashTable *private_ids;	/* Map of existing private IDs */
	volatile gint destroyed;	/* Whether this room has been destroyed */
	gboolean check_allowed;		/* Whether to check tokens when participants join (see below) */
	GHashTable *allowed;		/* Map of participants (as tokens) allowed to join */
	gboolean notify_joining;	/* Whether an event is sent to notify all participants if a new participant joins the room */
	janus_mutex mutex;			/* Mutex to lock this room instance */
	janus_refcount ref;			/* Reference counter for this room */

	GThread *thread;			/* Mixer thread for this room */
	unsigned min_audio_mix_participants;	/* Threshold for audio mix activation */
	volatile gint audio_mix_active;		/* Flag, indicating audio mix activity */
} janus_videoroom;
static GHashTable *rooms;
static janus_mutex rooms_mutex = JANUS_MUTEX_INITIALIZER;
static char *admin_key = NULL;
static gboolean lock_rtpfwd = FALSE;

typedef struct janus_videoroom_session {
	janus_plugin_session *handle;
	gint64 sdp_sessid;
	gint64 sdp_version;
	janus_videoroom_p_type participant_type;
	gpointer participant;
	volatile gint started;
	volatile gint dataready;
	volatile gint hangingup;
	volatile gint destroyed;
	janus_mutex mutex;
	janus_refcount ref;
} janus_videoroom_session;
static GHashTable *sessions;
static janus_mutex sessions_mutex = JANUS_MUTEX_INITIALIZER;

/* A host whose ports gets streamed RTP packets of the corresponding type */
typedef struct janus_videoroom_srtp_context janus_videoroom_srtp_context;
typedef struct janus_videoroom_rtp_forwarder {
	void *source;
	uint32_t stream_id;
	gboolean is_video;
	gboolean is_data;
	uint32_t ssrc;
	int payload_type;
	int substream;
	struct sockaddr_in serv_addr;
	struct sockaddr_in6 serv_addr6;
	/* Only needed for RTCP */
	int rtcp_fd;
	uint16_t local_rtcp_port, remote_rtcp_port;
	GSource *rtcp_recv;
	/* Only needed when forwarding simulcasted streams to a single endpoint */
	gboolean simulcast;
	janus_rtp_switching_context context;
	janus_rtp_simulcasting_context sim_context;
	/* Only needed for SRTP forwarders */
	gboolean is_srtp;
	janus_videoroom_srtp_context *srtp_ctx;
	/* Reference */
	volatile gint destroyed;
	janus_refcount ref;
} janus_videoroom_rtp_forwarder;
static void janus_videoroom_rtp_forwarder_destroy(janus_videoroom_rtp_forwarder *forward);
static void janus_videoroom_rtp_forwarder_free(const janus_refcount *f_ref);
/* SRTP encryption may be needed, and potentially shared */
struct janus_videoroom_srtp_context {
	GHashTable *contexts;
	char *id;
	srtp_t ctx;
	srtp_policy_t policy;
	char sbuf[1500];
	int slen;
	/* Keep track of how many forwarders are using this context */
	uint8_t count;
};
static void janus_videoroom_srtp_context_free(gpointer data);
/* RTCP support in RTP forwarders */
typedef struct janus_videoroom_rtcp_receiver {
	GSource parent;
	janus_videoroom_rtp_forwarder *forward;
	GDestroyNotify destroy;
} janus_videoroom_rtcp_receiver;
static void janus_videoroom_rtp_forwarder_rtcp_receive(janus_videoroom_rtp_forwarder *forward);
static gboolean janus_videoroom_rtp_forwarder_rtcp_prepare(GSource *source, gint *timeout) {
	*timeout = -1;
	return FALSE;
}
static gboolean janus_videoroom_rtp_forwarder_rtcp_dispatch(GSource *source, GSourceFunc callback, gpointer user_data) {
	janus_videoroom_rtcp_receiver *r = (janus_videoroom_rtcp_receiver *)source;
	/* Receive the packet */
	if(r)
		janus_videoroom_rtp_forwarder_rtcp_receive(r->forward);
	return G_SOURCE_CONTINUE;
}
static void janus_videoroom_publisher_stream_dereference_void(void *ps);
static void janus_videoroom_rtp_forwarder_rtcp_finalize(GSource *source) {
	janus_videoroom_rtcp_receiver *r = (janus_videoroom_rtcp_receiver *)source;
	/* Remove the reference to the forwarder */
	if(r && r->forward) {
		if(r->forward->source) {
			janus_videoroom_publisher_stream_dereference_void(r->forward->source);
			r->forward->source = NULL;
		}
		janus_refcount_decrease(&r->forward->ref);
	}
}
static GSourceFuncs janus_videoroom_rtp_forwarder_rtcp_funcs = {
	janus_videoroom_rtp_forwarder_rtcp_prepare,
	NULL,
	janus_videoroom_rtp_forwarder_rtcp_dispatch,
	janus_videoroom_rtp_forwarder_rtcp_finalize,
	NULL, NULL
};
static GMainContext *rtcpfwd_ctx = NULL;
static GMainLoop *rtcpfwd_loop = NULL;
static GThread *rtcpfwd_thread = NULL;
static void *janus_videoroom_rtp_forwarder_rtcp_thread(void *data);

#define MIN_MIX_PARTICIPANTS		1
#define AUDIO_LEVEL_REPORT_INTERVAL_MS	4000
#define MIX_SAMPLE_RATE			48000
static const char * g_audio_mix_dummy_publisher_name = "audiomix";

#define AUDIO_EVENT_REPORT_INTERVAL_MS	1000

typedef struct {
	speech_mixer_t speech_mixer;
	gint64 sdp_sessid;
	uint32_t audio_ssrc;
	guint32 subscriber_pvt_id;
} speech_mix_participant_t;

static void *speech_mix_thread(void *data);
static int speech_mix_participant_init(const janus_videoroom * room, speech_mix_participant_t * participant, const char * debug_tag);
static void speech_mix_participant_free(speech_mix_participant_t * participant);
static void speech_mix_incoming_rtp(janus_videoroom * room, speech_mix_participant_t *participant, char *buf, int len, const char * debug_tag);

static unsigned now_ms(void);

typedef struct janus_videoroom_stat {
	int packets_count;
	int bytes_count;
	unsigned time_start;
	unsigned time_last_packet;
	unsigned max_pause;
} janus_videoroom_stat;

static void janus_videoroom_stat_on_packet(janus_videoroom_stat * h, int bytes, unsigned now_ms) {
	h->packets_count++;
	h->bytes_count += bytes;
	if (!h->time_start) {
		h->time_start = now_ms;
	} else {
		unsigned pause = now_ms - h->time_last_packet;
		h->max_pause = MAX(pause, h->max_pause);
	}
	h->time_last_packet = now_ms;
}

static void janus_videoroom_stat_on_destroy(janus_videoroom_stat * h, unsigned now_ms) {
	unsigned audio_session_time = h[0].time_start ? now_ms - h[0].time_start : 0;
	unsigned video_session_time = h[1].time_start ? now_ms - h[1].time_start : 0;
	unsigned session_time = MAX(audio_session_time, video_session_time);
	unsigned audio_time = h[0].time_last_packet - h[0].time_start;
	unsigned video_time = h[1].time_last_packet - h[1].time_start;
	JANUS_LOG(LOG_INFO, "Session time = %.2f s  ", session_time*0.001f);
	if(h[1].packets_count)
		JANUS_LOG(LOG_INFO, " Video: %d bytes in %d packets for %.2f s (%d ms pause max)\n", h[1].bytes_count, h[1].packets_count, video_time*0.001f, h[1].max_pause);
	if(h[0].packets_count)
		JANUS_LOG(LOG_INFO, " Audio: %d bytes in %d packets for %.2f s (%d ms pause max)\n", h[0].bytes_count, h[0].packets_count, audio_time*0.001f, h[0].max_pause);
	else
		JANUS_LOG(LOG_INFO, " No audio!\n");
}

typedef struct janus_videoroom_publisher {
	janus_videoroom_session *session;
	janus_videoroom *room;	/* Room */
	guint64 room_id;	/* Unique room ID */
	gchar *room_id_str;	/* Unique room ID (when using strings) */
	guint64 user_id;	/* Unique ID in the room */
	gchar *user_id_str;	/* Unique ID in the room (when using strings) */
	guint32 pvt_id;		/* This is sent to the publisher for mapping purposes, but shouldn't be shared with others */
	gchar *display;		/* Display name (just for fun) */
	gboolean dummy;		/* Whether this is a dummy publisher used just for placeholder subscriptions */
	janus_audiocodec acodec;				/* Audio codec preference for this publisher (if audio) */
	janus_videocodec vcodec;				/* Video codec preference for this publisher (if video) */
	int user_audio_active_packets;	/* Participant's audio_active_packets overwriting global room setting */
	int user_audio_level_average;	/* Participant's audio_level_average overwriting global room setting */
	gboolean talking; 	/* Whether this participant is currently talking (uses audio levels extension) */
	gboolean firefox;	/* We send Firefox users a different kind of FIR */
	GList *streams;				/* List of media streams sent by this publisher (audio, video and/or data) */
	GHashTable *streams_byid;	/* As above, indexed by mindex */
	GHashTable *streams_bymid;	/* As above, indexed by mid */
	int data_mindex;			/* We keep track of the mindex for data, as there can only be one */
	janus_mutex streams_mutex;
	uint32_t bitrate;
	gint64 remb_startup;/* Incremental changes on REMB to reach the target at startup */
	gint64 remb_latest;	/* Time of latest sent REMB (to avoid flooding) */
	gboolean recording_active;	/* Whether this publisher has to be recorded or not */
	gchar *recording_base;	/* Base name for the recording (e.g., /path/to/filename, will generate /path/to/filename-audio.mjr and/or /path/to/filename-video.mjr) */
	janus_mutex rec_mutex;	/* Mutex to protect the recorders from race conditions */
	GSList *subscriptions;	/* Subscriptions this publisher has created (who this publisher is watching) */
	janus_mutex subscribers_mutex;
	janus_mutex own_subscriptions_mutex;
	GHashTable *srtp_contexts;	/* SRTP contexts that we can share among RTP forwarders */
	/* Index of RTP (or data) forwarders for this participant (all streams), if any */
	GHashTable *rtp_forwarders;
	janus_mutex rtp_forwarders_mutex;
	int udp_sock; /* The udp socket on which to forward rtp packets */
	gboolean kicked;	/* Whether this participant has been kicked */
	gboolean e2ee;		/* If media from this publisher is end-to-end encrypted */
	volatile gint destroyed;
	janus_refcount ref;
	speech_mix_participant_t speech_mix_participant;
	janus_videoroom_stat stat[2];
	gboolean want_mixed_audio;
	gboolean support_multitrack;
	int last_audio_level_reported;
	guint64 last_audio_level_report_time;
} janus_videoroom_publisher;
/* Each VideoRoom publisher can share multiple streams, so each stream is its own structure */
typedef struct janus_videoroom_publisher_stream {
	janus_videoroom_publisher *publisher;	/* Publisher instance this stream belongs to */
	janus_videoroom_media type;				/* Type of this stream (audio, video or data) */
	int mindex;								/* mindex of this stream */
	char *mid;								/* mid of this stream */
	char *description;						/* Description of this stream (user provided) */
	gboolean disabled;						/* Whether this stream is temporarily disabled or not */
	gboolean active;						/* Whether this stream is active or not */
	gboolean muted;							/* Whether this stream has been muted by a moderator */
	janus_audiocodec acodec;				/* Audio codec this publisher is using (if audio) */
	janus_videocodec vcodec;				/* Video codec this publisher is using (if video) */
	int pt;									/* Payload type of this stream (if audio or video) */
	char *fmtp;								/* fmtp that ended up being negotiated, if any (for video profiles) */
	char *h264_profile;						/* H264 profile used for this stream (if video and H264 codec) */
	char *vp9_profile;						/* VP9 profile this publisher is using (if video and VP9 codec) */
	guint32 ssrc;							/* Internal SSRC of this stream */
	gint64 fir_latest;						/* Time of latest sent PLI (to avoid flooding) */
	gint fir_seq;							/* FIR sequence number, if needed */
	gboolean opusfec;						/* Whether this stream is sending inband Opus FEC */
	gboolean opusdtx;						/* Whether this publisher is using Opus DTX (Discontinuous Transmission) */
	gboolean opusstereo;					/* Whether this publisher is doing stereo Opus */
	gboolean simulcast, svc;				/* Whether this stream uses simulcast or VP9 SVC */
	uint32_t vssrc[3];						/* Only needed in case VP8 (or H.264) simulcasting is involved */
	char *rid[3];							/* Only needed if simulcasting is rid-based */
	int rid_extmap_id;						/* rid extmap ID */
	janus_mutex rid_mutex;					/* Mutex to protect access to the rid array and the extmap ID */
	/* RTP extensions, if negotiated */
	guint8 audio_level_extmap_id;			/* Audio level extmap ID */
	guint8 video_orient_extmap_id;			/* Video orientation extmap ID */
	guint8 playout_delay_extmap_id;			/* Playout delay extmap ID */
	janus_sdp_mdirection audio_level_mdir, video_orient_mdir, playout_delay_mdir;
	/* Playout delays to enforce when relaying this stream, if the extension has been negotiated */
	int16_t min_delay, max_delay;
	/* Audio level processing, if enabled */
	int audio_dBov_level;					/* Value in dBov of the audio level (last value from extension) */
	int audio_active_packets;				/* Participant's number of audio packets to accumulate */
	int audio_dBov_sum;						/* Participant's accumulated dBov value for audio level */
	gboolean talking;						/* Whether this participant is currently talking (uses audio levels extension) */
	/* Recording related stuff, if enabled */
	janus_recorder *rc;
	janus_rtp_switching_context rec_ctx;
	janus_rtp_simulcasting_context rec_simctx;
	/* RTP (or data) forwarders for this stream, if any */
	GHashTable *rtp_forwarders;
	janus_mutex rtp_forwarders_mutex;
	/* Subscriptions to this publisher stream (who's receiving it)  */
	GSList *subscribers;
	janus_mutex subscribers_mutex;
	volatile gint destroyed;
	janus_refcount ref;
	guint64 last_talking_event_report_time;
} janus_videoroom_publisher_stream;
/* Helper to add a new RTP forwarder for a specific stream sent by publisher */
static janus_videoroom_rtp_forwarder *janus_videoroom_rtp_forwarder_add_helper(janus_videoroom_publisher *p,
	janus_videoroom_publisher_stream *ps,
	const gchar *host, int port, int rtcp_port, int pt, uint32_t ssrc,
	gboolean simulcast, int srtp_suite, const char *srtp_crypto,
	int substream, gboolean is_video, gboolean is_data);
static json_t *janus_videoroom_rtp_forwarder_summary(janus_videoroom_rtp_forwarder *f);
static void janus_videoroom_create_dummy_publisher(janus_videoroom *room, GHashTable *streams, const char * display, int pvt_id);

typedef struct janus_videoroom_subscriber {
	janus_videoroom_session *session;
	janus_videoroom *room;	/* Room */
	guint64 room_id;		/* Unique room ID */
	gchar *room_id_str;		/* Unique room ID (when using strings) */
	GList *streams;				/* List of media stream subscriptions originated by this subscriber (audio, video and/or data) */
	GHashTable *streams_byid;	/* As above, indexed by mindex */
	GHashTable *streams_bymid;	/* As above, indexed by mid */
	janus_mutex streams_mutex;
	gboolean close_pc;		/* Whether we should automatically close the PeerConnection when the publisher goes away */
	guint32 pvt_id;			/* Private ID of the participant that is subscribing (if available/provided) */
	gboolean paused;
	gboolean kicked;	/* Whether this subscription belongs to a participant that has been kicked */
	gboolean e2ee;		/* If media for this subscriber is end-to-end encrypted */
	volatile gint answered, pending_offer, pending_restart;
	volatile gint destroyed;
	janus_refcount ref;
	int serial_id;		/* Internal outgoing packet count (avoid sending same packet twice) */
	gboolean want_mixed_audio;
	gboolean have_audiomix_subscribtion;
	guint32 mixer_pvt_id;
	gchar *display;		/* Display name (just for fun) */
} janus_videoroom_subscriber;
/* Each VideoRoom subscriber can be subscribed to multiple streams, belonging to
 * the same or different publishers: as such, each stream is its own structure */
typedef struct janus_videoroom_subscriber_stream {
	janus_videoroom_subscriber *subscriber;			/* Subscriber instance this stream belongs to */
	GSList *publisher_streams;						/* Complete list of publisher streams (e.g., when this is data) */
	int mindex;				/* The media index of this stream (may not be the same as the publisher stream) */
	char *mid;				/* The mid of this stream (may not be the same as the publisher stream) */
	char *crossrefid;		/* An id provided while subscribing to uniquely identify the subscription in the list of subscriptions */
	gboolean send;			/* Whether this stream media must be sent to this subscriber */
	/* The following properties are copied from the source, in case this stream becomes inactive */
	janus_videoroom_media type;			/* Type of this stream (audio, video or data) */
	janus_audiocodec acodec;			/* Audio codec this publisher is using (if audio) */
	janus_videocodec vcodec;			/* Video codec this publisher is using (if video) */
	char *h264_profile;					/* H264 profile used for this stream (if video and H264 codec) */
	char *vp9_profile;					/* VP9 profile this publisher is using (if video and VP9 codec) */
	int pt;								/* Payload type of this stream (if audio or video) */
	gboolean opusfec;					/* Whether this stream is using inband Opus FEC */
	/* RTP and simulcasting contexts */
	janus_rtp_switching_context context;
	janus_rtp_simulcasting_context sim_context;
	janus_vp8_simulcast_context vp8_context;
	/* The following are only relevant if we're doing VP9 SVC, and are not to be confused with plain
	 * simulcast, which has similar info (substream/templayer) but in a completely different context */
	int spatial_layer, target_spatial_layer;
	gint64 last_spatial_layer[3];
	int temporal_layer, target_temporal_layer;
	/* Playout delays to enforce when relaying this stream, if the extension has been negotiated */
	int16_t min_delay, max_delay;
	volatile gint ready, destroyed;
	janus_refcount ref;
	janus_videoroom_stat stat[2];
	int relay_fail_count;
	int relay_fail_last_time;
} janus_videoroom_subscriber_stream;

typedef struct janus_videoroom_rtp_relay_packet {
	janus_videoroom_publisher_stream *source;
	janus_rtp_header *data;
	gint length;
	gboolean is_rtp;	/* This may be a data packet and not RTP */
	gboolean is_video;
	uint32_t ssrc[3];
	uint32_t timestamp;
	uint16_t seq_number;
	/* Extensions to add, if any */
	janus_plugin_rtp_extensions extensions;
	/* Whether simulcast is involved */
	gboolean simulcast;
	/* The following are only relevant if we're doing VP9 SVC*/
	gboolean svc;
	janus_vp9_svc_info svc_info;
	/* The following is only relevant for datachannels */
	gboolean textdata;
} janus_videoroom_rtp_relay_packet;

// https://stackoverflow.com/questions/63166/how-to-determine-cpu-and-memory-consumption-from-inside-a-process
static int parse_proce_status_line(char* line) {
	// This assumes that a digit will be found and the line ends in " Kb".
	int i = strlen(line);
	const char* p = line;
	while(*p <'0' || *p > '9') p++;
	line[i-3] = '\0';
	i = atoi(p);
	return i;
}

static int get_vm_used_kb(void) { //Note: this value is in KB!
	FILE* file = fopen("/proc/self/status", "r");
	if (!file)
		return -1;

	int result = -1;
	char line[128];
	_Static_assert(sizeof(line[0]) == 1, "WRONG SIZE");

	while(fgets(line, sizeof(line), file) != NULL) {
		if(strncmp(line, "VmSize:", 7) == 0) {
			result = parse_proce_status_line(line);
			break;
		}
	}
	fclose(file);
	return result;
}

static volatile gint g_rooms_count = 0;
static volatile gint g_publishers_count = 0;
static volatile gint g_subscribers_count = 0;
static volatile gint g_publishers_streams_count = 0;
static volatile gint g_subscribers_streams_count = 0;
static volatile gint g_sessions_count = 0;

static void print_resource_usage(const char * reason) {
	unsigned now = now_ms();
	static unsigned prev_print;
	if(now - prev_print > 10000 || reason) {
		// can't use g_hash_table_size(rooms) w/o locks
		JANUS_LOG(LOG_INFO, "USED: %8d kb %d rooms %d sessions, %d streams from %d publishers, %d streams for %d subscribers (%s)\n",
			get_vm_used_kb(),
			g_atomic_int_get(&g_rooms_count),
			g_atomic_int_get(&g_sessions_count),
			g_atomic_int_get(&g_publishers_streams_count), g_atomic_int_get(&g_publishers_count),
			g_atomic_int_get(&g_subscribers_streams_count), g_atomic_int_get(&g_subscribers_count),
			reason ? reason : "timer"
		);
		prev_print = now;
	}
}

/* Start / stop recording */
static void janus_videoroom_recorder_create(janus_videoroom_publisher_stream *ps);
static void janus_videoroom_recorder_close(janus_videoroom_publisher *participant);

static void janus_videoroom_subscriber_print_stat(janus_videoroom_subscriber *s) {
	if(s && s->room) {
		janus_mutex_lock(&s->streams_mutex);
		GList *temp = s->streams;
		guint streams_count = g_list_length(temp);
		janus_mutex_unlock(&s->streams_mutex);
		JANUS_LOG(LOG_INFO, "Statistics for subscriber %s (want_mix=%d) in room %s with %d streams:\n", s->display, (int)(s->want_mixed_audio), s->room_id_str, (int)streams_count);
	}
}

static void janus_videoroom_publisher_print_stat(janus_videoroom_publisher *p) {
	if(p) {
		JANUS_LOG(LOG_INFO, "Statistics for publisher %s in room %s:  ", p->display, p->room_id_str);
		janus_videoroom_stat_on_destroy(p->stat, now_ms());
	}
}

/* Freeing stuff */
static void janus_videoroom_subscriber_stream_destroy(janus_videoroom_subscriber_stream *s) {
	if(s && g_atomic_int_compare_and_exchange(&s->destroyed, 0, 1)) {
		janus_videoroom_publisher_stream * ps = (janus_videoroom_publisher_stream *)(s->publisher_streams?s->publisher_streams->data:NULL);
		JANUS_LOG(LOG_INFO, "Statistics for stream %p to %s from %s (ssrc %"SCNu32"): ", s, s->subscriber ? s->subscriber->display : "null", (ps && ps->publisher)?ps->publisher->display:"null", ps?ps->ssrc:0);
		janus_videoroom_stat_on_destroy(s->stat, now_ms());
		janus_refcount_decrease(&s->ref);
	}
	/* TODO Should unref the subscriber instance? */
}

static void janus_videoroom_subscriber_stream_unref(janus_videoroom_subscriber_stream *s) {
	/* Decrease the counter */
	if(s)
		janus_refcount_decrease(&s->ref);
}

static void janus_videoroom_subscriber_stream_free(const janus_refcount *s_ref) {
	janus_videoroom_subscriber_stream *s = janus_refcount_containerof(s_ref, janus_videoroom_subscriber_stream, ref);
	/* This subscriber stream can be destroyed, free all the resources */
		/* TODO Anything else we should free? */
	g_free(s->mid);
	g_free(s->crossrefid);
	g_free(s->h264_profile);
	g_free(s->vp9_profile);
	g_free(s);
	g_atomic_int_dec_and_test(&g_subscribers_streams_count);
}

static void janus_videoroom_subscriber_destroy(janus_videoroom_subscriber *s) {
	if(s && g_atomic_int_compare_and_exchange(&s->destroyed, 0, 1))
		janus_refcount_decrease(&s->ref);
}

static void janus_videoroom_subscriber_free(const janus_refcount *s_ref) {
	janus_videoroom_subscriber *s = janus_refcount_containerof(s_ref, janus_videoroom_subscriber, ref);
	/* This subscriber can be destroyed, free all the resources */
	g_free(s->room_id_str);
	g_free(s->display);
	/* TODO Get rid of all the streams */
	g_list_free_full(s->streams, (GDestroyNotify)(janus_videoroom_subscriber_stream_destroy));
	g_hash_table_unref(s->streams_byid);
	g_hash_table_unref(s->streams_bymid);
		/* TODO Unref the publisher stream? */

	g_free(s);
	g_atomic_int_dec_and_test(&g_subscribers_count);
}

static void janus_videoroom_publisher_stream_destroy(janus_videoroom_publisher_stream *ps) {
	if(ps && g_atomic_int_compare_and_exchange(&ps->destroyed, 0, 1)) {
		if(ps->publisher)
			janus_refcount_decrease(&ps->publisher->ref);
		ps->publisher = NULL;
		janus_refcount_decrease(&ps->ref);
	}
	/* TODO Should unref the publisher instance? */
}

static void janus_videoroom_publisher_stream_unref(janus_videoroom_publisher_stream *ps) {
	/* Decrease the counter */
	if(ps)
		janus_refcount_decrease(&ps->ref);
}
static void janus_videoroom_publisher_stream_dereference_void(void *ps) {
	janus_videoroom_publisher_stream_unref((janus_videoroom_publisher_stream *)ps);
}

static void janus_videoroom_publisher_stream_free(const janus_refcount *ps_ref) {
	janus_videoroom_publisher_stream *ps = janus_refcount_containerof(ps_ref, janus_videoroom_publisher_stream, ref);
	/* This publisher stream can be destroyed, free all the resources */
		/* TODO Anything else we should free? */
	g_free(ps->mid);
	g_free(ps->description);
	g_free(ps->fmtp);
	g_free(ps->h264_profile);
	g_free(ps->vp9_profile);
	janus_recorder_destroy(ps->rc);
	g_hash_table_destroy(ps->rtp_forwarders);
	ps->rtp_forwarders = NULL;
	janus_mutex_destroy(&ps->rtp_forwarders_mutex);
	g_slist_free(ps->subscribers);
	janus_mutex_destroy(&ps->subscribers_mutex);
	janus_mutex_destroy(&ps->rid_mutex);
	janus_rtp_simulcasting_cleanup(NULL, NULL, ps->rid, NULL);
	g_free(ps);
	g_atomic_int_dec_and_test(&g_publishers_streams_count);
}

static void janus_videoroom_publisher_dereference(janus_videoroom_publisher *p) {
	/* This is used by g_pointer_clear and g_hash_table_new_full so that NULL is only possible if that was inserted into the hash table. */
	janus_refcount_decrease(&p->ref);
}

static void janus_videoroom_publisher_dereference_nodebug(janus_videoroom_publisher *p) {
	janus_refcount_decrease_nodebug(&p->ref);
}

static void janus_videoroom_publisher_destroy(janus_videoroom_publisher *p) {
	if(p && g_atomic_int_compare_and_exchange(&p->destroyed, 0, 1)) {
		/* Forwarders with RTCP support may have an extra reference, stop their source */
		janus_mutex_lock(&p->rtp_forwarders_mutex);
		if(g_hash_table_size(p->rtp_forwarders) > 0) {
			janus_videoroom_publisher_stream *ps = NULL;
			GList *temp = p->streams;
			while(temp) {
				ps = (janus_videoroom_publisher_stream *)temp->data;
				janus_mutex_lock(&ps->rtp_forwarders_mutex);
				if(g_hash_table_size(ps->rtp_forwarders) == 0) {
					janus_mutex_unlock(&ps->rtp_forwarders_mutex);
					temp = temp->next;
					continue;
				}
				GHashTableIter iter_f;
				gpointer key_f, value_f;
				g_hash_table_iter_init(&iter_f, ps->rtp_forwarders);
				while(g_hash_table_iter_next(&iter_f, &key_f, &value_f)) {
					janus_videoroom_rtp_forwarder *rpv = value_f;
					if(rpv->rtcp_recv) {
						GSource *source = rpv->rtcp_recv;
						rpv->rtcp_recv = NULL;
						g_source_destroy(source);
						g_source_unref(source);
					}
				}
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
				temp = temp->next;
			}
		}
		janus_mutex_unlock(&p->rtp_forwarders_mutex);
		janus_refcount_decrease(&p->ref);
	}
}

static void janus_videoroom_publisher_free(const janus_refcount *p_ref) {
	janus_videoroom_publisher *p = janus_refcount_containerof(p_ref, janus_videoroom_publisher, ref);
	g_free(p->room_id_str);
	g_free(p->user_id_str);
	g_free(p->display);
	g_free(p->recording_base);
	/* Get rid of all the streams */
	g_list_free_full(p->streams, (GDestroyNotify)(janus_videoroom_publisher_stream_destroy));
	g_hash_table_unref(p->streams_byid);
	g_hash_table_unref(p->streams_bymid);

	if(p->udp_sock > 0)
		close(p->udp_sock);
	g_hash_table_destroy(p->rtp_forwarders);
	p->rtp_forwarders = NULL;
	g_hash_table_destroy(p->srtp_contexts);
	p->srtp_contexts = NULL;
	g_slist_free(p->subscriptions);
	speech_mix_participant_free(&p->speech_mix_participant);

	janus_mutex_destroy(&p->subscribers_mutex);
	janus_mutex_destroy(&p->rtp_forwarders_mutex);

	/* If this is a dummy publisher, get rid of the session too */
	// ASP: dummy publisher have a 'fake' incomplete session, which is crashed in destroy()
	if(p->dummy && p->session)
		janus_refcount_decrease(&p->session->ref);

	g_free(p);
	g_atomic_int_dec_and_test(&g_publishers_count);
}

static void janus_videoroom_session_destroy(janus_videoroom_session *session) {
	if(session && g_atomic_int_compare_and_exchange(&session->destroyed, 0, 1))
		janus_refcount_decrease(&session->ref);
}

static void janus_videoroom_session_free(const janus_refcount *session_ref) {
	janus_videoroom_session *session = janus_refcount_containerof(session_ref, janus_videoroom_session, ref);
	/* Remove the reference to the core plugin session */
	if(session->handle)
		janus_refcount_decrease(&session->handle->ref);
	/* This session can be destroyed, free all the resources */
	janus_mutex_destroy(&session->mutex);
	g_free(session);
	g_atomic_int_dec_and_test(&g_sessions_count);
}

static void janus_videoroom_room_dereference(janus_videoroom *room) {
	janus_refcount_decrease(&room->ref);
}

static void janus_videoroom_room_destroy(janus_videoroom *room) {
	if(room && g_atomic_int_compare_and_exchange(&room->destroyed, 0, 1)) {
		if (room->thread)
			g_thread_join(room->thread);
		janus_refcount_decrease(&room->ref);
	}
}

static void janus_videoroom_room_free(const janus_refcount *room_ref) {
	janus_videoroom *room = janus_refcount_containerof(room_ref, janus_videoroom, ref);
	/* This room can be destroyed, free all the resources */
	g_free(room->room_id_str);
	g_free(room->room_name);
	g_free(room->room_secret);
	g_free(room->room_pin);
	g_free(room->rec_dir);
	g_free(room->vp9_profile);
	g_free(room->h264_profile);
	g_hash_table_destroy(room->participants);
	g_hash_table_destroy(room->private_ids);
	g_hash_table_destroy(room->allowed);
	g_free(room);
	g_atomic_int_dec_and_test(&g_rooms_count);
	print_resource_usage("after room dtor");
}

static void janus_videoroom_message_free(janus_videoroom_message *msg) {
	if(!msg || msg == &exit_message)
		return;

	if(msg->handle && msg->handle->plugin_handle) {
		janus_videoroom_session *session = (janus_videoroom_session *)msg->handle->plugin_handle;
		janus_refcount_decrease(&session->ref);
	}
	msg->handle = NULL;

	g_free(msg->transaction);
	msg->transaction = NULL;
	if(msg->message)
		json_decref(msg->message);
	msg->message = NULL;
	if(msg->jsep)
		json_decref(msg->jsep);
	msg->jsep = NULL;

	g_free(msg);
}

static void janus_videoroom_codecstr(janus_videoroom *videoroom, char *audio_codecs, char *video_codecs, int str_len, const char *split) {
	if (audio_codecs) {
		audio_codecs[0] = 0;
		g_snprintf(audio_codecs, str_len, "%s", janus_audiocodec_name(videoroom->acodec[0]));
		if (videoroom->acodec[1] != JANUS_AUDIOCODEC_NONE) {
			janus_strlcat(audio_codecs, split, str_len);
			janus_strlcat(audio_codecs, janus_audiocodec_name(videoroom->acodec[1]), str_len);
		}
		if (videoroom->acodec[2] != JANUS_AUDIOCODEC_NONE) {
			janus_strlcat(audio_codecs, split, str_len);
			janus_strlcat(audio_codecs, janus_audiocodec_name(videoroom->acodec[2]), str_len);
		}
		if (videoroom->acodec[3] != JANUS_AUDIOCODEC_NONE) {
			janus_strlcat(audio_codecs, split, str_len);
			janus_strlcat(audio_codecs, janus_audiocodec_name(videoroom->acodec[3]), str_len);
		}
		if (videoroom->acodec[4] != JANUS_AUDIOCODEC_NONE) {
			janus_strlcat(audio_codecs, split, str_len);
			janus_strlcat(audio_codecs, janus_audiocodec_name(videoroom->acodec[4]), str_len);
		}
	}
	if (video_codecs) {
		video_codecs[0] = 0;
		g_snprintf(video_codecs, str_len, "%s", janus_videocodec_name(videoroom->vcodec[0]));
		if (videoroom->vcodec[1] != JANUS_VIDEOCODEC_NONE) {
			janus_strlcat(video_codecs, split, str_len);
			janus_strlcat(video_codecs, janus_videocodec_name(videoroom->vcodec[1]), str_len);
		}
		if (videoroom->vcodec[2] != JANUS_VIDEOCODEC_NONE) {
			janus_strlcat(video_codecs, split, str_len);
			janus_strlcat(video_codecs, janus_videocodec_name(videoroom->vcodec[2]), str_len);
		}
		if (videoroom->vcodec[3] != JANUS_VIDEOCODEC_NONE) {
			janus_strlcat(video_codecs, split, str_len);
			janus_strlcat(video_codecs, janus_videocodec_name(videoroom->vcodec[3]), str_len);
		}
		if (videoroom->vcodec[4] != JANUS_VIDEOCODEC_NONE) {
			janus_strlcat(video_codecs, split, str_len);
			janus_strlcat(video_codecs, janus_videocodec_name(videoroom->vcodec[4]), str_len);
		}
	}
}

static void janus_videoroom_reqpli(janus_videoroom_publisher_stream *ps, const char *reason) {
	if(ps == NULL || g_atomic_int_get(&ps->destroyed) || ps->publisher == NULL || g_atomic_int_get(&ps->publisher->destroyed))
		return;
	/* Send a PLI */
	JANUS_LOG(LOG_VERB, "%s sending PLI to %s (#%d, %s)\n", reason,
		ps->publisher->user_id_str, ps->mindex, ps->publisher->display ? ps->publisher->display : "??");
	gateway->send_pli_stream(ps->publisher->session->handle, ps->mindex);
	/* Update the time of when we last sent a keyframe request */
	ps->fir_latest = janus_get_monotonic_time();
}

/* Error codes */
#define JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR		499
#define JANUS_VIDEOROOM_ERROR_NO_MESSAGE		421
#define JANUS_VIDEOROOM_ERROR_INVALID_JSON		422
#define JANUS_VIDEOROOM_ERROR_INVALID_REQUEST	423
#define JANUS_VIDEOROOM_ERROR_JOIN_FIRST		424
#define JANUS_VIDEOROOM_ERROR_ALREADY_JOINED	425
#define JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM		426
#define JANUS_VIDEOROOM_ERROR_ROOM_EXISTS		427
#define JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED		428
#define JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT	429
#define JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT	430
#define JANUS_VIDEOROOM_ERROR_INVALID_SDP_TYPE	431
#define JANUS_VIDEOROOM_ERROR_PUBLISHERS_FULL	432
#define JANUS_VIDEOROOM_ERROR_UNAUTHORIZED		433
#define JANUS_VIDEOROOM_ERROR_ALREADY_PUBLISHED	434
#define JANUS_VIDEOROOM_ERROR_NOT_PUBLISHED		435
#define JANUS_VIDEOROOM_ERROR_ID_EXISTS			436
#define JANUS_VIDEOROOM_ERROR_INVALID_SDP		437


/* RTP forwarder helpers */
static janus_videoroom_rtp_forwarder *janus_videoroom_rtp_forwarder_add_helper(janus_videoroom_publisher *p,
		janus_videoroom_publisher_stream *ps,
		const gchar *host, int port, int rtcp_port, int pt, uint32_t ssrc,
		gboolean simulcast, int srtp_suite, const char *srtp_crypto,
		int substream, gboolean is_video, gboolean is_data) {
	if(!p || !ps || !host) {
		return NULL;
	}
	if(ipv6_disabled && strstr(host, ":") != NULL) {
		JANUS_LOG(LOG_ERR, "Attempt to create an IPv6 forwarder, but IPv6 networking is not available\n");
		return NULL;
	}
	janus_refcount_increase(&p->ref);
	janus_refcount_increase(&ps->ref);
	janus_mutex_lock(&ps->rtp_forwarders_mutex);
	/* Do we need to bind to a port for RTCP? */
	int fd = -1;
	uint16_t local_rtcp_port = 0;
	if(!is_data && rtcp_port > 0) {
		fd = socket(!ipv6_disabled ? AF_INET6 : AF_INET, SOCK_DGRAM, IPPROTO_UDP);
		if(fd < 0) {
			janus_mutex_unlock(&ps->rtp_forwarders_mutex);
			janus_refcount_decrease(&ps->ref);
			janus_refcount_decrease(&p->ref);
			JANUS_LOG(LOG_ERR, "Error creating RTCP socket for new RTP forwarder... %d (%s)\n",
				errno, g_strerror(errno));
			return NULL;
		}
		struct sockaddr *address = NULL;
		struct sockaddr_in addr4 = { 0 };
		struct sockaddr_in6 addr6 = { 0 };
		socklen_t len = 0;
		if(!ipv6_disabled) {
			/* Configure the socket so that it can be used both on IPv4 and IPv6 */
			int v6only = 0;
			if(setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) != 0) {
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
				janus_refcount_decrease(&ps->ref);
				janus_refcount_decrease(&p->ref);
				JANUS_LOG(LOG_ERR, "Error configuring RTCP socket for new RTP forwarder... %d (%s)\n",
					errno, g_strerror(errno));
				close(fd);
				return NULL;
			}
			len = sizeof(addr6);
			addr6.sin6_family = AF_INET6;
			addr6.sin6_port = htons(0);		/* The RTCP port we received is the remote one */
			addr6.sin6_addr = in6addr_any;
			address = (struct sockaddr *)&addr6;
		} else {
			/* IPv6 is disabled, only do IPv4 */
			len = sizeof(addr4);
			addr4.sin_family = AF_INET;
			addr4.sin_port = htons(0);		/* The RTCP port we received is the remote one */
			addr4.sin_addr.s_addr = INADDR_ANY;
			address = (struct sockaddr *)&addr4;
		}
		if(bind(fd, (struct sockaddr *)address, len) < 0 ||
				getsockname(fd, (struct sockaddr *)address, &len) < 0) {
			janus_mutex_unlock(&ps->rtp_forwarders_mutex);
			janus_refcount_decrease(&ps->ref);
			janus_refcount_decrease(&p->ref);
			JANUS_LOG(LOG_ERR, "Error binding RTCP socket for new RTP forwarder... %d (%s)\n",
				errno, g_strerror(errno));
			close(fd);
			return NULL;
		}
		local_rtcp_port = ntohs(!ipv6_disabled ? addr6.sin6_port : addr4.sin_port);
		JANUS_LOG(LOG_VERB, "Bound local %s RTCP port: %"SCNu16"\n",
			is_video ? "video" : "audio", local_rtcp_port);
	}
	janus_videoroom_rtp_forwarder *forward = g_malloc0(sizeof(janus_videoroom_rtp_forwarder));
	forward->source = ps;
	forward->rtcp_fd = fd;
	forward->local_rtcp_port = local_rtcp_port;
	forward->remote_rtcp_port = rtcp_port > 0 ? rtcp_port : 0;
	/* First of all, let's check if we need to setup an SRTP forwarder */
	if(!is_data && srtp_suite > 0 && srtp_crypto != NULL) {
		/* First of all, let's check if there's already an RTP forwarder with
		 * the same SRTP context: make sure SSRC and pt are the same too */
		char media[10] = {0};
		if(!is_video) {
			g_sprintf(media, "audio");
		} else if(is_video) {
			g_sprintf(media, "video%d", substream);
		}
		char srtp_id[256] = {0};
		g_snprintf(srtp_id, 255, "%s-%s-%"SCNu32"-%d", srtp_crypto, media, ssrc, pt);
		JANUS_LOG(LOG_VERB, "SRTP context ID: %s\n", srtp_id);
		janus_videoroom_srtp_context *srtp_ctx = g_hash_table_lookup(p->srtp_contexts, srtp_id);
		if(srtp_ctx != NULL) {
			JANUS_LOG(LOG_VERB, "  -- Reusing existing SRTP context\n");
			srtp_ctx->count++;
			forward->srtp_ctx = srtp_ctx;
		} else {
			/* Nope, base64 decode the crypto string and set it as a new SRTP context */
			JANUS_LOG(LOG_VERB, "  -- Creating new SRTP context\n");
			srtp_ctx = g_malloc0(sizeof(janus_videoroom_srtp_context));
			gsize len = 0;
			guchar *decoded = g_base64_decode(srtp_crypto, &len);
			if(len < SRTP_MASTER_LENGTH) {
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
			janus_refcount_decrease(&ps->ref);
			janus_refcount_decrease(&p->ref);
				JANUS_LOG(LOG_ERR, "Invalid SRTP crypto (%s)\n", srtp_crypto);
				g_free(decoded);
				g_free(srtp_ctx);
				if(forward->rtcp_fd > -1)
					close(forward->rtcp_fd);
				g_free(forward);
				return NULL;
			}
			/* Set SRTP policy */
			srtp_policy_t *policy = &srtp_ctx->policy;
			srtp_crypto_policy_set_rtp_default(&(policy->rtp));
			if(srtp_suite == 32) {
				srtp_crypto_policy_set_aes_cm_128_hmac_sha1_32(&(policy->rtp));
			} else if(srtp_suite == 80) {
				srtp_crypto_policy_set_aes_cm_128_hmac_sha1_80(&(policy->rtp));
			}
			policy->ssrc.type = ssrc_any_outbound;
			policy->key = decoded;
			policy->next = NULL;
			/* Create SRTP context */
			srtp_err_status_t res = srtp_create(&srtp_ctx->ctx, policy);
			if(res != srtp_err_status_ok) {
				/* Something went wrong... */
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
				janus_refcount_decrease(&ps->ref);
				janus_refcount_decrease(&p->ref);
				JANUS_LOG(LOG_ERR, "Error creating forwarder SRTP session: %d (%s)\n", res, janus_srtp_error_str(res));
				g_free(decoded);
				policy->key = NULL;
				g_free(srtp_ctx);
				if(forward->rtcp_fd > -1)
					close(forward->rtcp_fd);
				g_free(forward);
				return NULL;
			}
			srtp_ctx->contexts = p->srtp_contexts;
			srtp_ctx->id = g_strdup(srtp_id);
			srtp_ctx->count = 1;
			g_hash_table_insert(p->srtp_contexts, srtp_ctx->id, srtp_ctx);
			forward->srtp_ctx = srtp_ctx;
		}
		forward->is_srtp = TRUE;
	}
	forward->is_video = is_video;
	forward->payload_type = pt;
	forward->ssrc = ssrc;
	forward->substream = substream;
	forward->is_data = is_data;
	/* Check if the host address is IPv4 or IPv6 */
	if(strstr(host, ":") != NULL) {
		forward->serv_addr6.sin6_family = AF_INET6;
		inet_pton(AF_INET6, host, &(forward->serv_addr6.sin6_addr));
		forward->serv_addr6.sin6_port = htons(port);
	} else {
		forward->serv_addr.sin_family = AF_INET;
		inet_pton(AF_INET, host, &(forward->serv_addr.sin_addr));
		forward->serv_addr.sin_port = htons(port);
	}
	if(is_video && simulcast) {
		forward->simulcast = TRUE;
		janus_rtp_switching_context_reset(&forward->context);
		janus_rtp_simulcasting_context_reset(&forward->sim_context);
		forward->sim_context.rid_ext_id = ps->rid_extmap_id;
		forward->sim_context.substream_target = 2;
		forward->sim_context.templayer_target = 2;
	}
	janus_refcount_init(&forward->ref, janus_videoroom_rtp_forwarder_free);
	guint32 stream_id = janus_random_uint32();
	while(g_hash_table_lookup(ps->rtp_forwarders, GUINT_TO_POINTER(stream_id)) != NULL &&
			g_hash_table_lookup(p->rtp_forwarders, GUINT_TO_POINTER(stream_id)) != NULL) {
		stream_id = janus_random_uint32();
	}
	forward->stream_id = stream_id;
	g_hash_table_insert(ps->rtp_forwarders, GUINT_TO_POINTER(stream_id), forward);
	g_hash_table_insert(p->rtp_forwarders, GUINT_TO_POINTER(stream_id), GUINT_TO_POINTER(stream_id));
	if(fd > -1) {
		/* We need RTCP: track this file descriptor, and ref the forwarder */
		janus_refcount_increase(&ps->ref);
		janus_refcount_increase(&forward->ref);
		forward->rtcp_recv = g_source_new(&janus_videoroom_rtp_forwarder_rtcp_funcs, sizeof(janus_videoroom_rtcp_receiver));
		janus_videoroom_rtcp_receiver *rr = (janus_videoroom_rtcp_receiver *)forward->rtcp_recv;
		rr->forward = forward;
		g_source_set_priority(forward->rtcp_recv, G_PRIORITY_DEFAULT);
		g_source_add_unix_fd(forward->rtcp_recv, fd, G_IO_IN | G_IO_ERR);
		g_source_attach((GSource *)forward->rtcp_recv, rtcpfwd_ctx);
		/* Send a couple of empty RTP packets to the remote port to do latching */
		struct sockaddr *address = NULL;
		struct sockaddr_in addr4 = { 0 };
		struct sockaddr_in6 addr6 = { 0 };
		socklen_t addrlen = 0;
		if(forward->serv_addr.sin_family == AF_INET) {
			addr4.sin_family = AF_INET;
			addr4.sin_addr.s_addr = forward->serv_addr.sin_addr.s_addr;
			addr4.sin_port = htons(forward->remote_rtcp_port);
			address = (struct sockaddr *)&addr4;
			addrlen = sizeof(addr4);
		} else {
			addr6.sin6_family = AF_INET6;
			memcpy(&addr6.sin6_addr, &forward->serv_addr6.sin6_addr, sizeof(struct in6_addr));
			addr6.sin6_port = htons(forward->remote_rtcp_port);
			address = (struct sockaddr *)&addr6;
			addrlen = sizeof(addr6);
		}
		janus_rtp_header rtp;
		memset(&rtp, 0, sizeof(rtp));
		rtp.version = 2;
		(void)sendto(fd, &rtp, 12, 0, address, addrlen);
		(void)sendto(fd, &rtp, 12, 0, address, addrlen);
	}
	janus_mutex_unlock(&ps->rtp_forwarders_mutex);
	janus_refcount_decrease(&ps->ref);
	janus_refcount_decrease(&p->ref);
	JANUS_LOG(LOG_VERB, "Added %s/%d rtp_forward to participant %s host: %s:%d stream_id: %"SCNu32"\n",
		is_data ? "data" : (is_video ? "video" : "audio"), substream, p->user_id_str, host, port, stream_id);
	return forward;
}

static json_t *janus_videoroom_rtp_forwarder_summary(janus_videoroom_rtp_forwarder *f) {
	if(f == NULL)
		return NULL;
	json_t *json = json_object();
	json_object_set_new(json, "stream_id", json_integer(f->stream_id));
	char address[100];
	if(f->serv_addr.sin_family == AF_INET) {
		json_object_set_new(json, "host", json_string(
			inet_ntop(AF_INET, &f->serv_addr.sin_addr, address, sizeof(address))));
	} else {
		json_object_set_new(json, "host", json_string(
			inet_ntop(AF_INET6, &f->serv_addr6.sin6_addr, address, sizeof(address))));
	}
	json_object_set_new(json, "port", json_integer(ntohs(f->serv_addr.sin_port)));
	if(f->is_data) {
		json_object_set_new(json, "type", json_string("data"));
	} else if(f->is_video) {
		json_object_set_new(json, "type", json_string("video"));
		if(f->local_rtcp_port > 0)
			json_object_set_new(json, "local_rtcp_port", json_integer(f->local_rtcp_port));
		if(f->remote_rtcp_port > 0)
			json_object_set_new(json, "remote_rtcp_port", json_integer(f->remote_rtcp_port));
		if(f->payload_type)
			json_object_set_new(json, "pt", json_integer(f->payload_type));
		if(f->ssrc)
			json_object_set_new(json, "ssrc", json_integer(f->ssrc));
		if(f->substream)
			json_object_set_new(json, "substream", json_integer(f->substream));
	} else {
		json_object_set_new(json, "type", json_string("audio"));
		if(f->local_rtcp_port > 0)
			json_object_set_new(json, "local_rtcp_port", json_integer(f->local_rtcp_port));
		if(f->remote_rtcp_port > 0)
			json_object_set_new(json, "remote_rtcp_port", json_integer(f->remote_rtcp_port));
		if(f->payload_type)
			json_object_set_new(json, "pt", json_integer(f->payload_type));
		if(f->ssrc)
			json_object_set_new(json, "ssrc", json_integer(f->ssrc));
	}
	if(f->is_srtp)
		json_object_set_new(json, "srtp", json_true());
	return json;
}

static void janus_videoroom_rtp_forwarder_destroy(janus_videoroom_rtp_forwarder *forward) {
	if(forward && g_atomic_int_compare_and_exchange(&forward->destroyed, 0, 1)) {
		if(forward->rtcp_fd > -1 && forward->rtcp_recv != NULL) {
			g_source_destroy(forward->rtcp_recv);
			g_source_unref(forward->rtcp_recv);
		}
		janus_refcount_decrease(&forward->ref);
	}
}
static void janus_videoroom_rtp_forwarder_free(const janus_refcount *f_ref) {
	janus_videoroom_rtp_forwarder *forward = janus_refcount_containerof(f_ref, janus_videoroom_rtp_forwarder, ref);
	if(forward->rtcp_fd > -1)
		close(forward->rtcp_fd);
	if(forward->is_srtp && forward->srtp_ctx) {
		forward->srtp_ctx->count--;
		if(forward->srtp_ctx->count == 0 && forward->srtp_ctx->contexts != NULL)
			g_hash_table_remove(forward->srtp_ctx->contexts, forward->srtp_ctx->id);
	}
	g_free(forward);
	forward = NULL;
}

static void janus_videoroom_srtp_context_free(gpointer data) {
	if(data) {
		janus_videoroom_srtp_context *srtp_ctx = (janus_videoroom_srtp_context *)data;
		if(srtp_ctx) {
			g_free(srtp_ctx->id);
			srtp_dealloc(srtp_ctx->ctx);
			g_free(srtp_ctx->policy.key);
			g_free(srtp_ctx);
			srtp_ctx = NULL;
		}
	}
}

/* Helper to create a dummy publisher, with placeholder streams for each supported codec */
static void janus_videoroom_create_dummy_publisher(janus_videoroom *room, GHashTable *streams, const char * display, int pvt_id) {
	if(room == NULL || !room->dummy_publisher)
		return;
	/* We create a dummy session first, that's not actually bound to anything */
	janus_videoroom_session *session = g_malloc0(sizeof(janus_videoroom_session));
	session->handle = NULL;
	session->participant_type = janus_videoroom_p_type_publisher;
	g_atomic_int_set(&session->started, 1);
	janus_mutex_init(&session->mutex);
	janus_refcount_init(&session->ref, janus_videoroom_session_free);
	g_atomic_int_inc(&g_sessions_count);
	/* We actually create a publisher instance, which has no associated session but looks like it's publishing */
	janus_videoroom_publisher *publisher = g_malloc0(sizeof(janus_videoroom_publisher));
	publisher->session = session;
	session->participant = publisher;
	publisher->room_id = room->room_id;
	publisher->room_id_str = room->room_id_str ? g_strdup(room->room_id_str) : NULL;
	publisher->room = room;
	janus_refcount_increase(&room->ref);
	publisher->user_id = janus_random_uint64();
	char user_id_num[30];
	g_snprintf(user_id_num, sizeof(user_id_num), "%"SCNu64, publisher->user_id);
	publisher->user_id_str = g_strdup(user_id_num);
	publisher->display = g_strdup(display?display:"Dummy publisher");
	publisher->acodec = JANUS_AUDIOCODEC_NONE;
	publisher->vcodec = JANUS_VIDEOCODEC_NONE;
	publisher->dummy = TRUE;
	publisher->pvt_id = pvt_id;
	janus_mutex_init(&publisher->subscribers_mutex);
	janus_mutex_init(&publisher->own_subscriptions_mutex);
	publisher->streams_byid = g_hash_table_new_full(NULL, NULL,
		NULL, (GDestroyNotify)janus_videoroom_publisher_stream_unref);
	publisher->streams_bymid = g_hash_table_new_full(g_str_hash, g_str_equal,
		(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_publisher_stream_unref);
	janus_mutex_init(&publisher->streams_mutex);
	janus_mutex_init(&publisher->rtp_forwarders_mutex);
	publisher->rtp_forwarders = g_hash_table_new(NULL, NULL);
	publisher->srtp_contexts = g_hash_table_new_full(g_str_hash, g_str_equal, NULL, (GDestroyNotify)janus_videoroom_srtp_context_free);
	publisher->udp_sock = -1;
	g_atomic_int_set(&publisher->destroyed, 0);
	janus_refcount_init(&publisher->ref, janus_videoroom_publisher_free);
	g_atomic_int_inc(&g_publishers_count);
	/* Now we create a separate publisher stream for each supported codec in the room */
	janus_videoroom_publisher_stream *ps = NULL;
	int mindex = 0;
	int i=0;
	for(i=0; i<5; i++) {
		if(room->acodec[i] == JANUS_AUDIOCODEC_NONE)
			continue;
		JANUS_LOG(LOG_INFO, "Done: add the dummy publisher to the list with codec  %s \n", janus_audiocodec_name(room->acodec[i]));

		char *fmtp = streams ? g_hash_table_lookup(streams, janus_audiocodec_name(room->acodec[i])) : NULL;
		if(streams != NULL && fmtp == NULL) {
			/* This codec is not in the dummy streams list, skip it */
			continue;
		}
		ps = g_malloc0(sizeof(janus_videoroom_publisher_stream));
		ps->type = JANUS_VIDEOROOM_MEDIA_AUDIO;
		ps->mindex = mindex;
		char mid[5];
		g_snprintf(mid, sizeof(mid), "%d", mindex);
		ps->mid = g_strdup(mid);
		ps->publisher = publisher;
		janus_refcount_increase(&publisher->ref);	/* Add a reference to the publisher */
		ps->active = TRUE;
		ps->acodec = room->acodec[i];
		ps->vcodec = JANUS_VIDEOCODEC_NONE;
		ps->pt = janus_audiocodec_pt(ps->acodec);
		if(fmtp != NULL && strcmp(fmtp, "none")) {
			/* Parse the fmtp string to see what we support (opus only) */
			if(ps->acodec == JANUS_AUDIOCODEC_OPUS) {
				if(strstr(fmtp, "useinbandfec=1"))
					ps->opusfec = TRUE;
				if(strstr(fmtp, "usedtx=1"))
					ps->opusdtx = TRUE;
				if(strstr(fmtp, "stereo=1"))
					ps->opusstereo = TRUE;
			}
		}
		ps->min_delay = -1;
		ps->max_delay = -1;
		g_atomic_int_set(&ps->destroyed, 0);
		janus_refcount_init(&ps->ref, janus_videoroom_publisher_stream_free);
		g_atomic_int_inc(&g_publishers_streams_count);
		janus_refcount_increase(&ps->ref);	/* This is for the id-indexed hashtable */
		janus_refcount_increase(&ps->ref);	/* This is for the mid-indexed hashtable */
		janus_mutex_init(&ps->subscribers_mutex);
		janus_mutex_init(&ps->rtp_forwarders_mutex);
		janus_mutex_init(&ps->rid_mutex);
		ps->rtp_forwarders = g_hash_table_new_full(NULL, NULL, NULL, (GDestroyNotify)janus_videoroom_rtp_forwarder_destroy);
		publisher->streams = g_list_append(publisher->streams, ps);
		g_hash_table_insert(publisher->streams_byid, GINT_TO_POINTER(ps->mindex), ps);
		g_hash_table_insert(publisher->streams_bymid, g_strdup(ps->mid), ps);
		mindex++;
	}
	for(i=0; i<5; i++) {
		if(room->vcodec[i] == JANUS_VIDEOCODEC_NONE)
			continue;
		char *fmtp = streams ? g_hash_table_lookup(streams, janus_videocodec_name(room->vcodec[i])) : NULL;
		if(streams != NULL && fmtp == NULL) {
			/* This codec is not in the dummy streams list, skip it */
			continue;
		}
		ps = g_malloc0(sizeof(janus_videoroom_publisher_stream));
		ps->type = JANUS_VIDEOROOM_MEDIA_VIDEO;
		ps->mindex = mindex;
		char mid[5];
		g_snprintf(mid, sizeof(mid), "%d", mindex);
		ps->mid = g_strdup(mid);
		ps->publisher = publisher;
		janus_refcount_increase(&publisher->ref);	/* Add a reference to the publisher */
		ps->active = TRUE;
		ps->acodec = JANUS_AUDIOCODEC_NONE;
		ps->vcodec = room->vcodec[i];
		ps->pt = janus_videocodec_pt(ps->vcodec);
		if(fmtp != NULL && strcmp(fmtp, "none")) {
			/* Parse the fmtp string to see what we support (H.264 and VP9 profiles only) */
			if(ps->vcodec == JANUS_VIDEOCODEC_H264)
				ps->h264_profile = janus_sdp_get_video_profile(ps->vcodec, fmtp);
			else if(ps->vcodec == JANUS_VIDEOCODEC_VP9) {
				ps->vp9_profile = janus_sdp_get_video_profile(ps->vcodec, fmtp);
			}
		}
		if(ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile == NULL && room->h264_profile != NULL)
			ps->h264_profile = g_strdup(room->h264_profile);
		else if(ps->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile == NULL && room->vp9_profile != NULL)
			ps->vp9_profile = g_strdup(room->vp9_profile);
		ps->min_delay = -1;
		ps->max_delay = -1;
		g_atomic_int_set(&ps->destroyed, 0);
		janus_refcount_init(&ps->ref, janus_videoroom_publisher_stream_free);
		g_atomic_int_inc(&g_publishers_streams_count);
		janus_refcount_increase(&ps->ref);	/* This is for the id-indexed hashtable */
		janus_refcount_increase(&ps->ref);	/* This is for the mid-indexed hashtable */
		janus_mutex_init(&ps->subscribers_mutex);
		janus_mutex_init(&ps->rtp_forwarders_mutex);
		janus_mutex_init(&ps->rid_mutex);
		ps->rtp_forwarders = g_hash_table_new_full(NULL, NULL, NULL, (GDestroyNotify)janus_videoroom_rtp_forwarder_destroy);
		publisher->streams = g_list_append(publisher->streams, ps);
		g_hash_table_insert(publisher->streams_byid, GINT_TO_POINTER(ps->mindex), ps);
		g_hash_table_insert(publisher->streams_bymid, g_strdup(ps->mid), ps);
		mindex++;
	}

	speech_mix_participant_init(room, &publisher->speech_mix_participant, publisher->display);

	/* Done: add the dummy publisher to the list */
	JANUS_LOG(LOG_INFO, "Done: add the dummy publisher to the list %s  %p\n", publisher->display, publisher->streams);
	janus_refcount_increase(&publisher->ref);
	g_hash_table_insert(room->participants,
		string_ids ? (gpointer)g_strdup(publisher->user_id_str) : (gpointer)janus_uint64_dup(publisher->user_id),
		publisher);
}

/* Helpers for subscription streams */
static janus_videoroom_subscriber_stream *janus_videoroom_subscriber_stream_add(janus_videoroom_subscriber *subscriber,
		janus_videoroom_publisher_stream *ps, const char *crossrefid,
		gboolean legacy, gboolean do_audio, gboolean do_video, gboolean do_data) {
	/* If this is a legacy subscription ("feed"), use the deprecated properties */
	if(legacy && ((ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO && !do_audio) ||
			(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && !do_video) ||
			(ps->type == JANUS_VIDEOROOM_MEDIA_DATA && !do_data))) {
		/* Skip this */
		JANUS_LOG(LOG_WARN, "Skipping %s stream (legacy subscription)\n", janus_videoroom_media_str(ps->type));
		return NULL;
	}
	if(ps->disabled)
		return NULL;
	/* Allocate a new subscriber stream instance */
	janus_videoroom_subscriber_stream *stream = g_malloc0(sizeof(janus_videoroom_subscriber_stream));
	stream->mindex = g_list_length(subscriber->streams);
	stream->crossrefid = g_strdup(crossrefid);
	stream->subscriber = subscriber;
	stream->publisher_streams = g_slist_append(stream->publisher_streams, ps);
	/* Copy properties from the source */
	stream->type = ps->type;
	stream->acodec = ps->acodec;
	stream->vcodec = ps->vcodec;
	if(stream->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile) {
		stream->h264_profile = g_strdup(ps->h264_profile);
	} else if(stream->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile) {
		stream->vp9_profile = g_strdup(ps->vp9_profile);
	}
	stream->pt = ps->pt;
	stream->opusfec = ps->opusfec;
	stream->min_delay = -1;
	stream->max_delay = -1;
	char mid[5];
	g_snprintf(mid, sizeof(mid), "%d", stream->mindex);
	stream->mid = g_strdup(mid);
	subscriber->streams = g_list_append(subscriber->streams, stream);
	g_hash_table_insert(subscriber->streams_byid, GINT_TO_POINTER(stream->mindex), stream);
	g_hash_table_insert(subscriber->streams_bymid, g_strdup(stream->mid), stream);
	/* Initialize the stream */
	janus_rtp_switching_context_reset(&stream->context);
	stream->send = TRUE;
	g_atomic_int_set(&stream->destroyed, 0);
	janus_refcount_init(&stream->ref, janus_videoroom_subscriber_stream_free);
	g_atomic_int_inc(&g_subscribers_streams_count);
	janus_refcount_increase(&stream->ref);	/* This is for the mid-indexed hashtable */
	janus_rtp_simulcasting_context_reset(&stream->sim_context);
	stream->sim_context.rid_ext_id = ps->rid_extmap_id;
	stream->sim_context.substream_target = 2;
	stream->sim_context.templayer_target = 2;
	janus_vp8_simulcast_context_reset(&stream->vp8_context);
	/* This stream may belong to a room where VP9 SVC has been enabled,
	 * let's assume we're interested in all layers for the time being */
	stream->spatial_layer = -1;
	stream->target_spatial_layer = 1;		/* FIXME Chrome sends 0 and 1 */
	stream->temporal_layer = -1;
	stream->target_temporal_layer = 2;	/* FIXME Chrome sends 0, 1 and 2 */
	janus_mutex_lock(&ps->subscribers_mutex);
	ps->subscribers = g_slist_append(ps->subscribers, stream);
	/* The two streams reference each other */
	janus_refcount_increase(&stream->ref);
	janus_refcount_increase(&ps->ref);
	janus_mutex_unlock(&ps->subscribers_mutex);
	return stream;
}

static janus_videoroom_subscriber_stream *janus_videoroom_subscriber_stream_add_or_replace(janus_videoroom_subscriber *subscriber,
		janus_videoroom_publisher_stream *ps, const char *crossrefid) {
	if(subscriber == NULL || ps == NULL)
		return NULL;
	/* First of all, let's check if there's an m-line we can reuse */
	gboolean found = FALSE;
	janus_videoroom_subscriber_stream *stream = NULL;
	GList *temp = subscriber->streams;
	while(temp) {
		stream = (janus_videoroom_subscriber_stream *)temp->data;
		janus_videoroom_publisher_stream *stream_ps = stream->publisher_streams ? stream->publisher_streams->data : NULL;
		if(stream_ps != NULL && stream_ps->type == ps->type && stream->type == JANUS_VIDEOROOM_MEDIA_DATA) {
			/* We already have a datachannel m-line, no need for others: just update the subscribers list */
			janus_mutex_lock(&ps->subscribers_mutex);
			if(g_slist_find(ps->subscribers, stream) == NULL && g_slist_find(stream->publisher_streams, ps) == NULL) {
				ps->subscribers = g_slist_append(ps->subscribers, stream);
				stream->publisher_streams = g_slist_append(stream->publisher_streams, ps);
				/* The two streams reference each other */
				janus_refcount_increase(&stream->ref);
				janus_refcount_increase(&ps->ref);
			}
			janus_mutex_unlock(&ps->subscribers_mutex);
			return NULL;
		}
		if(stream_ps == NULL && stream->type == ps->type) {
			/* There's an empty m-line of the right type, check if codecs match */
			if(stream->type == JANUS_VIDEOROOM_MEDIA_DATA ||
					(stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO && stream->acodec == ps->acodec) ||
					(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && stream->vcodec == ps->vcodec)) {
				found = TRUE;
				JANUS_LOG(LOG_VERB, "Reusing m-line %d for this subscription\n", stream->mindex);
				stream->opusfec = ps->opusfec;
				janus_rtp_simulcasting_context_reset(&stream->sim_context);
				if(ps->simulcast) {
					stream->sim_context.rid_ext_id = ps->rid_extmap_id;
					stream->sim_context.substream_target = 2;
					stream->sim_context.templayer_target = 2;
				}
				janus_vp8_simulcast_context_reset(&stream->vp8_context);
				if(ps->svc) {
					/* This stream belongs to a room where VP9 SVC has been enabled,
					 * let's assume we're interested in all layers for the time being */
					stream->spatial_layer = -1;
					stream->target_spatial_layer = 1;		/* FIXME Chrome sends 0 and 1 */
					stream->temporal_layer = -1;
					stream->target_temporal_layer = 2;	/* FIXME Chrome sends 0, 1 and 2 */
				}
				janus_mutex_lock(&ps->subscribers_mutex);
				if(g_slist_find(ps->subscribers, stream) == NULL && g_slist_find(stream->publisher_streams, ps) == NULL) {
					ps->subscribers = g_slist_append(ps->subscribers, stream);
					stream->publisher_streams = g_slist_append(stream->publisher_streams, ps);
					/* The two streams reference each other */
					janus_refcount_increase(&stream->ref);
					janus_refcount_increase(&ps->ref);
				}
				janus_mutex_unlock(&ps->subscribers_mutex);
				break;
			}
		}
		temp = temp->next;
	}
	if(found)  {
		g_free(stream->crossrefid);
		stream->crossrefid = g_strdup(crossrefid);
		return stream;
	}
	/* We couldn't find any, add a new one */
	return janus_videoroom_subscriber_stream_add(subscriber, ps, crossrefid, FALSE, FALSE, FALSE, FALSE);
}

static void janus_videoroom_subscriber_stream_remove(janus_videoroom_subscriber_stream *s,
		janus_videoroom_publisher_stream *ps, gboolean lock_ps) {
	if(ps != NULL) {
		/* Unsubscribe from this stream in particular (datachannels can have multiple sources) */
		if(g_slist_find(s->publisher_streams, ps) != NULL) {

			JANUS_LOG(LOG_INFO, "Statistics for stream %p to %s from %s (ssrc %"SCNu32"): ", s, s->subscriber ? s->subscriber->display : "null", (ps && ps->publisher)?ps->publisher->display:"null", ps?ps->ssrc:0);
			janus_videoroom_stat_on_destroy(s->stat, now_ms());

			/* Remove the subscription from the list of recipients */
			if(lock_ps)
				janus_mutex_lock(&ps->subscribers_mutex);
			gboolean unref_ps = FALSE, unref_ss = FALSE;
			if(g_slist_find(s->publisher_streams, ps) != NULL) {
				s->publisher_streams = g_slist_remove(s->publisher_streams, ps);
				unref_ps = TRUE;
				if(s->publisher_streams == NULL)
					g_atomic_int_set(&s->ready, 0);
			}
			s->opusfec = FALSE;
			if(g_slist_find(ps->subscribers, s) != NULL) {
				ps->subscribers = g_slist_remove(ps->subscribers, s);
				unref_ss = TRUE;
			}
			if(lock_ps)
				janus_mutex_unlock(&ps->subscribers_mutex);
			/* Unref the two streams, as they're not related anymore */
			if(unref_ps)
				janus_refcount_decrease(&ps->ref);
			if(unref_ss)
				janus_refcount_decrease(&s->ref);
		}
	} else {
		/* Unsubscribe from all sources (which will be one for audio/video, potentially more for datachannels) */
		while(s->publisher_streams) {
			ps = s->publisher_streams->data;
			janus_videoroom_subscriber_stream_remove(s, ps, lock_ps);
		}
	}
}

static json_t *janus_videoroom_subscriber_streams_summary(janus_videoroom_subscriber *subscriber, gboolean legacy, json_t *event) {
	json_t *media = json_array();
	GList *temp = subscriber->streams;
	while(temp) {
		janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)temp->data;
		janus_refcount_increase(&stream->ref);
		janus_videoroom_publisher_stream *ps = stream->publisher_streams ? stream->publisher_streams->data : NULL;
		if(ps)
			janus_refcount_increase(&ps->ref);
		json_t *m = json_object();
		json_object_set_new(m, "type", json_string(janus_videoroom_media_str(stream->type)));
		json_object_set_new(m, "active", (ps || stream->type == JANUS_VIDEOROOM_MEDIA_DATA) ? json_true() : json_false());
		json_object_set_new(m, "mindex", json_integer(stream->mindex));
		json_object_set_new(m, "mid", json_string(stream->mid));
		if(stream->crossrefid)
			json_object_set_new(m, "crossrefid", json_string(stream->crossrefid));
		json_object_set_new(m, "ready", g_atomic_int_get(&stream->ready) ? json_true() : json_false());
		json_object_set_new(m, "send", stream->send ? json_true() : json_false());
		if(ps && stream->type == JANUS_VIDEOROOM_MEDIA_DATA) {
			json_object_set_new(m, "sources", json_integer(g_slist_length(stream->publisher_streams)));
			json_t *ids = json_array();
			GSList *temp = stream->publisher_streams;
			janus_videoroom_publisher_stream *dps = NULL;
			while(temp) {
				dps = (janus_videoroom_publisher_stream *)temp->data;
				if(dps && dps->publisher)
					json_array_append_new(ids, string_ids ? json_string(dps->publisher->user_id_str) : json_integer(dps->publisher->user_id));
				temp = temp->next;
			}
			json_object_set_new(m, "source_ids", ids);
		} else if(ps && stream->type != JANUS_VIDEOROOM_MEDIA_DATA) {
			if(ps->publisher) {
				json_object_set_new(m, "feed_id", string_ids ? json_string(ps->publisher->user_id_str) : json_integer(ps->publisher->user_id));
				if(ps->publisher->display)
					json_object_set_new(m, "feed_display", json_string(ps->publisher->display));
				/* If this is a legacy subscription, put the info in the generic part too */
				if(legacy && event) {
					json_object_set_new(event, "id", string_ids ? json_string(ps->publisher->user_id_str) : json_integer(ps->publisher->user_id));
					if(ps->publisher->display)
						json_object_set_new(event, "display", json_string(ps->publisher->display));
				}
			}
			if(ps->mid)
				json_object_set_new(m, "feed_mid", json_string(ps->mid));
			if(ps->description)
				json_object_set_new(m, "feed_description", json_string(ps->description));
			if(stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
				json_object_set_new(m, "codec", json_string(janus_audiocodec_name(stream->acodec)));
			} else if(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
				json_object_set_new(m, "codec", json_string(janus_videocodec_name(stream->vcodec)));
				if(stream->vcodec == JANUS_VIDEOCODEC_H264 && stream->h264_profile != NULL)
					json_object_set_new(m, "h264-profile", json_string(stream->h264_profile));
				if(stream->vcodec == JANUS_VIDEOCODEC_VP9 && stream->vp9_profile != NULL)
					json_object_set_new(m, "vp9-profile", json_string(stream->vp9_profile));
				if(stream->min_delay > -1 && stream->max_delay > -1) {
					json_t *pd = json_object();
					json_object_set_new(pd, "min-delay", json_integer(stream->min_delay));
					json_object_set_new(pd, "max-delay", json_integer(stream->max_delay));
					json_object_set_new(m, "playout-delay", pd);
				}
			}
			if(ps->simulcast) {
				json_t *simulcast = json_object();
				json_object_set_new(simulcast, "substream", json_integer(stream->sim_context.substream));
				json_object_set_new(simulcast, "substream-target", json_integer(stream->sim_context.substream_target));
				json_object_set_new(simulcast, "temporal-layer", json_integer(stream->sim_context.templayer));
				json_object_set_new(simulcast, "temporal-layer-target", json_integer(stream->sim_context.templayer_target));
				if(stream->sim_context.drop_trigger > 0)
					json_object_set_new(simulcast, "fallback", json_integer(stream->sim_context.drop_trigger));
				json_object_set_new(m, "simulcast", simulcast);
			}
			if(ps->svc) {
				json_t *svc = json_object();
				json_object_set_new(svc, "spatial-layer", json_integer(stream->spatial_layer));
				json_object_set_new(svc, "target-spatial-layer", json_integer(stream->target_spatial_layer));
				json_object_set_new(svc, "temporal-layer", json_integer(stream->temporal_layer));
				json_object_set_new(svc, "target-temporal-layer", json_integer(stream->target_temporal_layer));
				json_object_set_new(m, "svc", svc);
			}
		}
		if(ps)
			janus_refcount_decrease(&ps->ref);
		janus_refcount_decrease(&stream->ref);
		json_array_append_new(media, m);
		temp = temp->next;
	}
	return media;
}

/* Helper to generate a new offer with the subscriber streams */
static json_t *janus_videoroom_subscriber_offer(janus_videoroom_subscriber *subscriber) {
	g_atomic_int_set(&subscriber->answered, 0);
	char s_name[100], audio_fmtp[256];
	g_snprintf(s_name, sizeof(s_name), "VideoRoom %s", subscriber->room->room_id_str);
	janus_sdp *offer = janus_sdp_generate_offer(s_name, "0.0.0.0",
		JANUS_SDP_OA_DONE);
	GList *temp = subscriber->streams;
	while(temp) {
		janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)temp->data;
		janus_videoroom_publisher_stream *ps = stream->publisher_streams ? stream->publisher_streams->data : NULL;
		int pt = -1;
		const char *codec = NULL;
		audio_fmtp[0] = '\0';
		if(ps && stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
			if(ps->opusfec)
				g_snprintf(audio_fmtp, sizeof(audio_fmtp), "useinbandfec=1");
			if(ps->opusdtx) {
				if(strlen(audio_fmtp) == 0) {
					g_snprintf(audio_fmtp, sizeof(audio_fmtp), "usedtx=1");
				} else {
					janus_strlcat(audio_fmtp, ";usedtx=1", sizeof(audio_fmtp));
				}
			}
			if(ps->opusstereo) {
				if(strlen(audio_fmtp) == 0) {
					g_snprintf(audio_fmtp, sizeof(audio_fmtp), "stereo=1");
				} else {
					janus_strlcat(audio_fmtp, ";stereo=1", sizeof(audio_fmtp));
				}
			}
		}
		if(stream->type != JANUS_VIDEOROOM_MEDIA_DATA) {
			pt = stream->pt;
			codec = (stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO ?
				janus_audiocodec_name(stream->acodec) : janus_videocodec_name(stream->vcodec));
		}
		janus_sdp_generate_offer_mline(offer,
			JANUS_SDP_OA_MLINE, janus_videoroom_media_sdptype(stream->type),
			JANUS_SDP_OA_MID, stream->mid,
			JANUS_SDP_OA_PT, pt,
			JANUS_SDP_OA_CODEC, codec,
			JANUS_SDP_OA_FMTP, (stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO && strlen(audio_fmtp) ? audio_fmtp : NULL),
			JANUS_SDP_OA_H264_PROFILE, (stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO ? stream->h264_profile : NULL),
			JANUS_SDP_OA_VP9_PROFILE, (stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO ? stream->vp9_profile : NULL),
			JANUS_SDP_OA_DIRECTION, ((ps && !ps->disabled) || stream->type == JANUS_VIDEOROOM_MEDIA_DATA) ? JANUS_SDP_SENDONLY : JANUS_SDP_INACTIVE,
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_AUDIO_LEVEL,
				(stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO && (ps && ps->audio_level_extmap_id > 0)) ? janus_rtp_extension_id(JANUS_RTP_EXTMAP_AUDIO_LEVEL) : 0,
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_MID, janus_rtp_extension_id(JANUS_RTP_EXTMAP_MID),
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_VIDEO_ORIENTATION,
				(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && (ps && ps->video_orient_extmap_id > 0)) ? janus_rtp_extension_id(JANUS_RTP_EXTMAP_VIDEO_ORIENTATION) : 0,
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_PLAYOUT_DELAY,
				(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && (ps && ps->playout_delay_extmap_id > 0)) ? janus_rtp_extension_id(JANUS_RTP_EXTMAP_PLAYOUT_DELAY) : 0,
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_TRANSPORT_WIDE_CC,
				(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && subscriber->room->transport_wide_cc_ext) ? janus_rtp_extension_id(JANUS_RTP_EXTMAP_TRANSPORT_WIDE_CC) : 0,
			JANUS_SDP_OA_EXTENSION, JANUS_RTP_EXTMAP_ABS_SEND_TIME,
				(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO) ? janus_rtp_extension_id(JANUS_RTP_EXTMAP_ABS_SEND_TIME) : 0,
			/* TODO Add other properties from original SDP */
			JANUS_SDP_OA_DONE);
		temp = temp->next;
	}
	/* Update (or set) the SDP version */
	subscriber->session->sdp_version++;
	offer->o_version = subscriber->session->sdp_version;
	char *sdp = janus_sdp_write(offer);
	janus_sdp_destroy(offer);
	json_t *jsep = json_pack("{ssss}", "type", "offer", "sdp", sdp);
	g_free(sdp);
	/* Done */
	return jsep;
}


/* Plugin implementation */
int janus_videoroom_init(janus_callbacks *callback, const char *config_path) {
	if(g_atomic_int_get(&stopping)) {
		/* Still stopping from before */
		return -1;
	}
	if(callback == NULL || config_path == NULL) {
		/* Invalid arguments */
		return -1;
	}

	/* Read configuration */
	char filename[255];
	g_snprintf(filename, 255, "%s/%s.jcfg", config_path, JANUS_VIDEOROOM_PACKAGE);
	JANUS_LOG(LOG_VERB, "Configuration file: %s\n", filename);
	config = janus_config_parse(filename);
	if(config == NULL) {
		JANUS_LOG(LOG_WARN, "Couldn't find .jcfg configuration file (%s), trying .cfg\n", JANUS_VIDEOROOM_PACKAGE);
		g_snprintf(filename, 255, "%s/%s.cfg", config_path, JANUS_VIDEOROOM_PACKAGE);
		JANUS_LOG(LOG_VERB, "Configuration file: %s\n", filename);
		config = janus_config_parse(filename);
	}
	config_folder = config_path;
	if(config != NULL)
		janus_config_print(config);

	sessions = g_hash_table_new_full(NULL, NULL, NULL, (GDestroyNotify)janus_videoroom_session_destroy);
	messages = g_async_queue_new_full((GDestroyNotify) janus_videoroom_message_free);

	/* This is the callback we'll need to invoke to contact the Janus core */
	gateway = callback;

	/* Parse configuration to populate the rooms list */
	if(config != NULL) {
		janus_config_category *config_general = janus_config_get_create(config, NULL, janus_config_type_category, "general");
		/* Any admin key to limit who can "create"? */
		janus_config_item *key = janus_config_get(config, config_general, janus_config_type_item, "admin_key");
		if(key != NULL && key->value != NULL)
			admin_key = g_strdup(key->value);
		janus_config_item *lrf = janus_config_get(config, config_general, janus_config_type_item, "lock_rtp_forward");
		if(admin_key && lrf != NULL && lrf->value != NULL)
			lock_rtpfwd = janus_is_true(lrf->value);
		janus_config_item *events = janus_config_get(config, config_general, janus_config_type_item, "events");
		if(events != NULL && events->value != NULL)
			notify_events = janus_is_true(events->value);
		if(!notify_events && callback->events_is_enabled()) {
			JANUS_LOG(LOG_WARN, "Notification of events to handlers disabled for %s\n", JANUS_VIDEOROOM_NAME);
		}
		janus_config_item *ids = janus_config_get(config, config_general, janus_config_type_item, "string_ids");
		if(ids != NULL && ids->value != NULL)
			string_ids = janus_is_true(ids->value);
		if(string_ids) {
			JANUS_LOG(LOG_INFO, "VideoRoom will use alphanumeric IDs, not numeric\n");
		}
	}
	rooms = g_hash_table_new_full(string_ids ? g_str_hash : g_int64_hash, string_ids ? g_str_equal : g_int64_equal,
		(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_room_destroy);
	/* Iterate on all rooms */
	if(config != NULL) {
		GList *clist = janus_config_get_categories(config, NULL), *cl = clist;
		while(cl != NULL) {
			janus_config_category *cat = (janus_config_category *)cl->data;
			if(cat->name == NULL || !strcasecmp(cat->name, "general")) {
				cl = cl->next;
				continue;
			}
			JANUS_LOG(LOG_VERB, "Adding VideoRoom room '%s'\n", cat->name);
			janus_config_item *desc = janus_config_get(config, cat, janus_config_type_item, "description");
			janus_config_item *priv = janus_config_get(config, cat, janus_config_type_item, "is_private");
			janus_config_item *secret = janus_config_get(config, cat, janus_config_type_item, "secret");
			janus_config_item *pin = janus_config_get(config, cat, janus_config_type_item, "pin");
			janus_config_item *req_pvtid = janus_config_get(config, cat, janus_config_type_item, "require_pvtid");
			janus_config_item *signed_tokens = janus_config_get(config, cat, janus_config_type_item, "signed_tokens");
			janus_config_item *bitrate = janus_config_get(config, cat, janus_config_type_item, "bitrate");
			janus_config_item *bitrate_cap = janus_config_get(config, cat, janus_config_type_item, "bitrate_cap");
			janus_config_item *maxp = janus_config_get(config, cat, janus_config_type_item, "publishers");
			janus_config_item *firfreq = janus_config_get(config, cat, janus_config_type_item, "fir_freq");
			janus_config_item *audiocodec = janus_config_get(config, cat, janus_config_type_item, "audiocodec");
			janus_config_item *videocodec = janus_config_get(config, cat, janus_config_type_item, "videocodec");
			janus_config_item *vp9profile = janus_config_get(config, cat, janus_config_type_item, "vp9_profile");
			janus_config_item *h264profile = janus_config_get(config, cat, janus_config_type_item, "h264_profile");
			janus_config_item *fec = janus_config_get(config, cat, janus_config_type_item, "opus_fec");
			janus_config_item *dtx = janus_config_get(config, cat, janus_config_type_item, "opus_dtx");
			janus_config_item *svc = janus_config_get(config, cat, janus_config_type_item, "video_svc");
			janus_config_item *audiolevel_ext = janus_config_get(config, cat, janus_config_type_item, "audiolevel_ext");
			janus_config_item *audiolevel_event = janus_config_get(config, cat, janus_config_type_item, "audiolevel_event");
			janus_config_item *audio_active_packets = janus_config_get(config, cat, janus_config_type_item, "audio_active_packets");
			janus_config_item *audio_level_average = janus_config_get(config, cat, janus_config_type_item, "audio_level_average");
			janus_config_item *videoorient_ext = janus_config_get(config, cat, janus_config_type_item, "videoorient_ext");
			janus_config_item *playoutdelay_ext = janus_config_get(config, cat, janus_config_type_item, "playoutdelay_ext");
			janus_config_item *transport_wide_cc_ext = janus_config_get(config, cat, janus_config_type_item, "transport_wide_cc_ext");
			janus_config_item *notify_joining = janus_config_get(config, cat, janus_config_type_item, "notify_joining");
			janus_config_item *req_e2ee = janus_config_get(config, cat, janus_config_type_item, "require_e2ee");
			janus_config_item *dummy_pub = janus_config_get(config, cat, janus_config_type_item, "dummy_publisher");
			janus_config_item *dummy_str = janus_config_get(config, cat, janus_config_type_array, "dummy_streams");
			janus_config_item *record = janus_config_get(config, cat, janus_config_type_item, "record");
			janus_config_item *rec_dir = janus_config_get(config, cat, janus_config_type_item, "rec_dir");
			janus_config_item *lock_record = janus_config_get(config, cat, janus_config_type_item, "lock_record");
			/* Create the video room */
			janus_videoroom *videoroom = g_malloc0(sizeof(janus_videoroom));
			const char *room_num = cat->name;
			if(strstr(room_num, "room-") == room_num)
				room_num += 5;
			if(!string_ids) {
				videoroom->room_id = g_ascii_strtoull(room_num, NULL, 0);
				if(videoroom->room_id == 0) {
					JANUS_LOG(LOG_ERR, "Can't add the VideoRoom room, invalid ID 0...\n");
					g_free(videoroom);
					cl = cl->next;
					continue;
				}
				/* Make sure the ID is completely numeric */
				char room_id_str[30];
				g_snprintf(room_id_str, sizeof(room_id_str), "%"SCNu64, videoroom->room_id);
				if(strcmp(room_num, room_id_str)) {
					JANUS_LOG(LOG_ERR, "Can't add the VideoRoom room, ID '%s' is not numeric...\n", room_num);
					g_free(videoroom);
					cl = cl->next;
					continue;
				}
			}
			/* Let's make sure the room doesn't exist already */
			janus_mutex_lock(&rooms_mutex);
			if(g_hash_table_lookup(rooms, string_ids ? (gpointer)room_num : (gpointer)&videoroom->room_id) != NULL) {
				/* It does... */
				janus_mutex_unlock(&rooms_mutex);
				JANUS_LOG(LOG_ERR, "Can't add the VideoRoom room, room %s already exists...\n", room_num);
				g_free(videoroom);
				cl = cl->next;
				continue;
			}
			janus_mutex_unlock(&rooms_mutex);
			videoroom->room_id_str = g_strdup(room_num);
			char *description = NULL;
			if(desc != NULL && desc->value != NULL && strlen(desc->value) > 0)
				description = g_strdup(desc->value);
			else
				description = g_strdup(cat->name);
			videoroom->room_name = description;
			if(secret != NULL && secret->value != NULL) {
				videoroom->room_secret = g_strdup(secret->value);
			}
			if(pin != NULL && pin->value != NULL) {
				videoroom->room_pin = g_strdup(pin->value);
			}
			videoroom->is_private = priv && priv->value && janus_is_true(priv->value);
			videoroom->require_pvtid = req_pvtid && req_pvtid->value && janus_is_true(req_pvtid->value);
			if(signed_tokens && signed_tokens->value && janus_is_true(signed_tokens->value)) {
				if(!gateway->auth_is_signed()) {
					JANUS_LOG(LOG_WARN, "Can't enforce signed tokens for this room, signed-mode not in use in the core\n");
				} else {
					videoroom->signed_tokens = TRUE;
				}
			}
			videoroom->require_e2ee = req_e2ee && req_e2ee->value && janus_is_true(req_e2ee->value);
			videoroom->max_publishers = 3;	/* FIXME How should we choose a default? */
			if(maxp != NULL && maxp->value != NULL)
				videoroom->max_publishers = atol(maxp->value);
			if(videoroom->max_publishers < 0)
				videoroom->max_publishers = 3;	/* FIXME How should we choose a default? */
			videoroom->min_audio_mix_participants = MIN_MIX_PARTICIPANTS;
			videoroom->audio_mix_active = FALSE;
			videoroom->bitrate = 0;
			if(bitrate != NULL && bitrate->value != NULL)
				videoroom->bitrate = atol(bitrate->value);
			if(videoroom->bitrate > 0 && videoroom->bitrate < 64000)
				videoroom->bitrate = 64000;	/* Don't go below 64k */
			videoroom->bitrate_cap = bitrate_cap && bitrate_cap->value && janus_is_true(bitrate_cap->value);
			videoroom->fir_freq = 0;
			if(firfreq != NULL && firfreq->value != NULL)
				videoroom->fir_freq = atol(firfreq->value);
			/* By default, we force Opus as the only audio codec */
			videoroom->acodec[0] = JANUS_AUDIOCODEC_OPUS;
			videoroom->acodec[1] = JANUS_AUDIOCODEC_NONE;
			videoroom->acodec[2] = JANUS_AUDIOCODEC_NONE;
			videoroom->acodec[3] = JANUS_AUDIOCODEC_NONE;
			videoroom->acodec[4] = JANUS_AUDIOCODEC_NONE;
			/* Check if we're forcing a different single codec, or allowing more than one */
			if(audiocodec && audiocodec->value) {
				gchar **list = g_strsplit(audiocodec->value, ",", 6);
				gchar *codec = list[0];
				if(codec != NULL) {
					int i=0;
					while(codec != NULL) {
						if(i == 5) {
							JANUS_LOG(LOG_WARN, "Ignoring extra audio codecs: %s\n", codec);
							break;
						}
						if(strlen(codec) > 0)
							videoroom->acodec[i] = janus_audiocodec_from_name(codec);
						i++;
						codec = list[i];
					}
				}
				g_clear_pointer(&list, g_strfreev);
			}
			/* By default, we force VP8 as the only video codec */
			videoroom->vcodec[0] = JANUS_VIDEOCODEC_VP8;
			videoroom->vcodec[1] = JANUS_VIDEOCODEC_NONE;
			videoroom->vcodec[2] = JANUS_VIDEOCODEC_NONE;
			videoroom->vcodec[3] = JANUS_VIDEOCODEC_NONE;
			videoroom->vcodec[4] = JANUS_VIDEOCODEC_NONE;
			/* Check if we're forcing a different single codec, or allowing more than one */
			if(videocodec && videocodec->value) {
				gchar **list = g_strsplit(videocodec->value, ",", 6);
				gchar *codec = list[0];
				if(codec != NULL) {
					int i=0;
					while(codec != NULL) {
						if(i == 5) {
							JANUS_LOG(LOG_WARN, "Ignoring extra video codecs: %s\n", codec);
							break;
						}
						if(strlen(codec) > 0)
							videoroom->vcodec[i] = janus_videocodec_from_name(codec);
						i++;
						codec = list[i];
					}
				}
				g_clear_pointer(&list, g_strfreev);
			}
			if(vp9profile && vp9profile->value && (videoroom->vcodec[0] == JANUS_VIDEOCODEC_VP9 ||
					videoroom->vcodec[1] == JANUS_VIDEOCODEC_VP9 ||
					videoroom->vcodec[2] == JANUS_VIDEOCODEC_VP9 ||
					videoroom->vcodec[3] == JANUS_VIDEOCODEC_VP9 ||
					videoroom->vcodec[4] == JANUS_VIDEOCODEC_VP9)) {
				videoroom->vp9_profile = g_strdup(vp9profile->value);
			}
			if(h264profile && h264profile->value && (videoroom->vcodec[0] == JANUS_VIDEOCODEC_H264 ||
					videoroom->vcodec[1] == JANUS_VIDEOCODEC_H264 ||
					videoroom->vcodec[2] == JANUS_VIDEOCODEC_H264 ||
					videoroom->vcodec[3] == JANUS_VIDEOCODEC_H264 ||
					videoroom->vcodec[4] == JANUS_VIDEOCODEC_H264)) {
				videoroom->h264_profile = g_strdup(h264profile->value);
			}
			videoroom->do_opusfec = TRUE;
			if(fec && fec->value) {
				videoroom->do_opusfec = janus_is_true(fec->value);
				if(videoroom->acodec[0] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[1] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[2] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[3] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[4] != JANUS_AUDIOCODEC_OPUS) {
					videoroom->do_opusfec = FALSE;
					JANUS_LOG(LOG_WARN, "Inband FEC is only supported for rooms that allow Opus: disabling it...\n");
				}
			}
			if(dtx && dtx->value) {
				videoroom->do_opusdtx = janus_is_true(dtx->value);
				if(videoroom->acodec[0] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[1] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[2] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[3] != JANUS_AUDIOCODEC_OPUS &&
						videoroom->acodec[4] != JANUS_AUDIOCODEC_OPUS) {
					videoroom->do_opusdtx = FALSE;
					JANUS_LOG(LOG_WARN, "DTX is only supported for rooms that allow Opus: disabling it...\n");
				}
			}
			if(svc && svc->value && janus_is_true(svc->value)) {
				if(videoroom->vcodec[0] == JANUS_VIDEOCODEC_VP9 &&
						videoroom->vcodec[1] == JANUS_VIDEOCODEC_NONE &&
						videoroom->vcodec[2] == JANUS_VIDEOCODEC_NONE &&
						videoroom->vcodec[3] == JANUS_VIDEOCODEC_NONE &&
						videoroom->vcodec[4] == JANUS_VIDEOCODEC_NONE) {
					videoroom->do_svc = TRUE;
				} else {
					JANUS_LOG(LOG_WARN, "SVC is only supported, in an experimental way, for VP9 only rooms: disabling it...\n");
				}
			}
			videoroom->audiolevel_ext = TRUE;
			if(audiolevel_ext != NULL && audiolevel_ext->value != NULL)
				videoroom->audiolevel_ext = janus_is_true(audiolevel_ext->value);
			videoroom->audiolevel_event = FALSE;
			if(audiolevel_event != NULL && audiolevel_event->value != NULL)
				videoroom->audiolevel_event = janus_is_true(audiolevel_event->value);
			if(videoroom->audiolevel_event) {
				videoroom->audio_active_packets = 100;
				if(audio_active_packets != NULL && audio_active_packets->value != NULL){
					if(atoi(audio_active_packets->value) > 0) {
						videoroom->audio_active_packets = atoi(audio_active_packets->value);
					} else {
						JANUS_LOG(LOG_WARN, "Invalid audio_active_packets value, using default: %d\n", videoroom->audio_active_packets);
					}
				}
				videoroom->audio_level_average = 25;
				if(audio_level_average != NULL && audio_level_average->value != NULL) {
					if(atoi(audio_level_average->value) > 0) {
						videoroom->audio_level_average = atoi(audio_level_average->value);
					} else {
						JANUS_LOG(LOG_WARN, "Invalid audio_level_average value provided, using default: %d\n", videoroom->audio_level_average);
					}
				}
			}
			videoroom->videoorient_ext = TRUE;
			if(videoorient_ext != NULL && videoorient_ext->value != NULL)
				videoroom->videoorient_ext = janus_is_true(videoorient_ext->value);
			videoroom->playoutdelay_ext = TRUE;
			if(playoutdelay_ext != NULL && playoutdelay_ext->value != NULL)
				videoroom->playoutdelay_ext = janus_is_true(playoutdelay_ext->value);
			videoroom->transport_wide_cc_ext = TRUE;
			if(transport_wide_cc_ext != NULL && transport_wide_cc_ext->value != NULL)
				videoroom->transport_wide_cc_ext = janus_is_true(transport_wide_cc_ext->value);
			if(record && record->value) {
				videoroom->record = janus_is_true(record->value);
			}
			if(rec_dir && rec_dir->value) {
				videoroom->rec_dir = g_strdup(rec_dir->value);
			}
			if(lock_record && lock_record->value) {
				videoroom->lock_record = janus_is_true(lock_record->value);
			}
			/* By default, the VideoRoom plugin does not notify about participants simply joining the room.
				It only notifies when the participant actually starts publishing media. */
			videoroom->notify_joining = FALSE;
			if(notify_joining != NULL && notify_joining->value != NULL)
				videoroom->notify_joining = janus_is_true(notify_joining->value);
			g_atomic_int_set(&videoroom->destroyed, 0);
			janus_mutex_init(&videoroom->mutex);
			janus_refcount_init(&videoroom->ref, janus_videoroom_room_free);
			print_resource_usage("before room ctor");
			g_atomic_int_inc(&g_rooms_count);
			videoroom->participants = g_hash_table_new_full(string_ids ? g_str_hash : g_int64_hash, string_ids ? g_str_equal : g_int64_equal,
				(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_publisher_dereference);
			videoroom->private_ids = g_hash_table_new(NULL, NULL);
			videoroom->check_allowed = FALSE;	/* Static rooms can't have an "allowed" list yet, no hooks to the configuration file */
			videoroom->allowed = g_hash_table_new_full(g_str_hash, g_str_equal, (GDestroyNotify)g_free, NULL);
			/* Should we create a dummy participant for placeholder m-lines? */
			if(dummy_pub && dummy_pub->value && janus_is_true(dummy_pub->value)) {
				videoroom->dummy_publisher = TRUE;
				/* Check if we only need a subset of codecs, and&/or a specific fmtp */
				GHashTable *dummy_streams = NULL;
				if(dummy_str != NULL) {
					GList *l = dummy_str->list;
					while(l) {
						janus_config_item *m = (janus_config_item *)l->data;
						if(m == NULL || m->type != janus_config_type_category) {
							JANUS_LOG(LOG_WARN, "  -- Invalid dummy stream item (not a category?), skipping in '%s'...\n", cat->name);
							l = l->next;
							continue;
						}
						janus_config_item *codec = janus_config_get(config, m, janus_config_type_item, "codec");
						if(codec == NULL || codec->value == NULL) {
							JANUS_LOG(LOG_WARN, "  -- Invalid dummy stream codec, skipping in '%s'...\n", cat->name);
							l = l->next;
							continue;
						}
						janus_config_item *fmtp = janus_config_get(config, m, janus_config_type_item, "fmtp");
						if(fmtp != NULL && fmtp->value == NULL) {
							JANUS_LOG(LOG_WARN, "  -- Invalid dummy stream fmtp, skipping in '%s'...\n", cat->name);
							l = l->next;
							continue;
						}
						if(dummy_streams == NULL)
							dummy_streams = g_hash_table_new_full(g_str_hash, g_str_equal, (GDestroyNotify)g_free, (GDestroyNotify)g_free);
						g_hash_table_insert(dummy_streams, g_strdup(codec->value), g_strdup(fmtp ? fmtp->value : "none"));
						l = l->next;
					}
				}
				/* Create the dummy publisher */
				janus_videoroom_create_dummy_publisher(videoroom, dummy_streams, NULL, 0);
				if(dummy_streams != NULL)
					g_hash_table_destroy(dummy_streams);
			}
			janus_mutex_lock(&rooms_mutex);
			g_hash_table_insert(rooms,
				string_ids ? (gpointer)g_strdup(videoroom->room_id_str) : (gpointer)janus_uint64_dup(videoroom->room_id),
				videoroom);
			janus_mutex_unlock(&rooms_mutex);
			/* Compute a list of the supported codecs for the summary */
			char audio_codecs[100], video_codecs[100];
			janus_videoroom_codecstr(videoroom, audio_codecs, video_codecs, sizeof(audio_codecs), "|");
			JANUS_LOG(LOG_VERB, "Created VideoRoom: %s (%s, %s, %s/%s codecs, secret: %s, pin: %s, pvtid: %s)\n",
				videoroom->room_id_str, videoroom->room_name,
				videoroom->is_private ? "private" : "public",
				audio_codecs, video_codecs,
				videoroom->room_secret ? videoroom->room_secret : "no secret",
				videoroom->room_pin ? videoroom->room_pin : "no pin",
				videoroom->require_pvtid ? "required" : "optional");
			if(videoroom->record) {
				JANUS_LOG(LOG_VERB, "  -- Room is going to be recorded in %s\n",
					videoroom->rec_dir ? videoroom->rec_dir : "the current folder");
			}
			if(videoroom->require_e2ee) {
				JANUS_LOG(LOG_VERB, "  -- All publishers MUST use end-to-end encryption\n");
			}
			if(videoroom->dummy_publisher) {
				JANUS_LOG(LOG_VERB, "  -- The room is going to have a dummy publisher for placeholder subscriptions\n");
			}

			char tname[32];
			g_snprintf(tname, sizeof(tname), "audiomix%"SCNu64, videoroom->room_id);
			GError *error = NULL;
			janus_refcount_increase(&videoroom->ref);
			videoroom->thread = g_thread_try_new(tname, &speech_mix_thread, videoroom, &error);

			cl = cl->next;
		}
		/* Done: we keep the configuration file open in case we get a "create" or "destroy" with permanent=true */
	}

	/* Show available rooms */
	janus_mutex_lock(&rooms_mutex);
	GHashTableIter iter;
	gpointer value;
	g_hash_table_iter_init(&iter, rooms);
	while (g_hash_table_iter_next(&iter, NULL, &value)) {
		janus_videoroom *vr = value;
		/* Compute a list of the supported codecs for the summary */
		char audio_codecs[100], video_codecs[100];
		janus_videoroom_codecstr(vr, audio_codecs, video_codecs, sizeof(audio_codecs), "|");
		JANUS_LOG(LOG_VERB, "  ::: [%s][%s] %"SCNu32", max %d publishers, FIR frequency of %d seconds, %s audio codec(s), %s video codec(s)\n",
			vr->room_id_str, vr->room_name, vr->bitrate, vr->max_publishers, vr->fir_freq,
			audio_codecs, video_codecs);
	}
	janus_mutex_unlock(&rooms_mutex);

	/* Thread for handling incoming RTCP packets from RTP forwarders, if any */
	rtcpfwd_ctx = g_main_context_new();
	rtcpfwd_loop = g_main_loop_new(rtcpfwd_ctx, FALSE);
	GError *error = NULL;
	rtcpfwd_thread = g_thread_try_new("videoroom rtcpfwd", janus_videoroom_rtp_forwarder_rtcp_thread, NULL, &error);
	if(error != NULL) {
		/* We show the error but it's not fatal */
		JANUS_LOG(LOG_ERR, "Got error %d (%s) trying to launch the VideoRoom RTCP thread for RTP forwarders...\n",
			error->code, error->message ? error->message : "??");
		g_error_free(error);
	}

	/* Finally, let's check if IPv6 is disabled, as we may need to know for forwarders */
	int fd = socket(AF_INET6, SOCK_DGRAM, IPPROTO_UDP);
	if(fd <= 0) {
		ipv6_disabled = TRUE;
	} else {
		int v6only = 0;
		if(setsockopt(fd, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) != 0)
			ipv6_disabled = TRUE;
	}
	if(fd > 0)
		close(fd);
	if(ipv6_disabled) {
		JANUS_LOG(LOG_WARN, "IPv6 disabled, will only create VideoRoom forwarders to IPv4 addresses\n");
	}

	g_atomic_int_set(&initialized, 1);

	/* Launch the thread that will handle incoming messages */
	error = NULL;
	handler_thread = g_thread_try_new("videoroom handler", janus_videoroom_handler, NULL, &error);
	if(error != NULL) {
		g_atomic_int_set(&initialized, 0);
		JANUS_LOG(LOG_ERR, "Got error %d (%s) trying to launch the VideoRoom handler thread...\n",
			error->code, error->message ? error->message : "??");
		g_error_free(error);
		janus_config_destroy(config);
		return -1;
	}
	JANUS_LOG(LOG_INFO, "%s initialized!\n", JANUS_VIDEOROOM_NAME);
	return 0;
}

void janus_videoroom_destroy(void) {
	if(!g_atomic_int_get(&initialized))
		return;
	g_atomic_int_set(&stopping, 1);

	g_async_queue_push(messages, &exit_message);
	if(handler_thread != NULL) {
		g_thread_join(handler_thread);
		handler_thread = NULL;
	}
	if(rtcpfwd_thread != NULL) {
		if(g_main_loop_is_running(rtcpfwd_loop)) {
			g_main_loop_quit(rtcpfwd_loop);
			g_main_context_wakeup(rtcpfwd_ctx);
		}
		g_thread_join(rtcpfwd_thread);
		rtcpfwd_thread = NULL;
	}

	/* FIXME We should destroy the sessions cleanly */
	janus_mutex_lock(&sessions_mutex);
	g_hash_table_destroy(sessions);
	sessions = NULL;
	janus_mutex_unlock(&sessions_mutex);

	janus_mutex_lock(&rooms_mutex);
	g_hash_table_destroy(rooms);
	rooms = NULL;
	janus_mutex_unlock(&rooms_mutex);

	g_async_queue_unref(messages);
	messages = NULL;

	janus_config_destroy(config);
	g_free(admin_key);

	g_atomic_int_set(&initialized, 0);
	g_atomic_int_set(&stopping, 0);
	JANUS_LOG(LOG_INFO, "%s destroyed!\n", JANUS_VIDEOROOM_NAME);
}

int janus_videoroom_get_api_compatibility(void) {
	/* Important! This is what your plugin MUST always return: don't lie here or bad things will happen */
	return JANUS_PLUGIN_API_VERSION;
}

int janus_videoroom_get_version(void) {
	return JANUS_VIDEOROOM_VERSION;
}

const char *janus_videoroom_get_version_string(void) {
	return JANUS_VIDEOROOM_VERSION_STRING;
}

const char *janus_videoroom_get_description(void) {
	return JANUS_VIDEOROOM_DESCRIPTION;
}

const char *janus_videoroom_get_name(void) {
	return JANUS_VIDEOROOM_NAME;
}

const char *janus_videoroom_get_author(void) {
	return JANUS_VIDEOROOM_AUTHOR;
}

const char *janus_videoroom_get_package(void) {
	return JANUS_VIDEOROOM_PACKAGE;
}

static janus_videoroom_session *janus_videoroom_lookup_session(janus_plugin_session *handle) {
	janus_videoroom_session *session = NULL;
	if (g_hash_table_contains(sessions, handle)) {
		session = (janus_videoroom_session *)handle->plugin_handle;
	}
	return session;
}

void janus_videoroom_create_session(janus_plugin_session *handle, int *error) {
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized)) {
		*error = -1;
		return;
	}
	janus_videoroom_session *session = g_malloc0(sizeof(janus_videoroom_session));
	session->handle = handle;
	session->participant_type = janus_videoroom_p_type_none;
	session->participant = NULL;
	g_atomic_int_set(&session->hangingup, 0);
	g_atomic_int_set(&session->destroyed, 0);
	handle->plugin_handle = session;
	janus_mutex_init(&session->mutex);
	janus_refcount_init(&session->ref, janus_videoroom_session_free);
	g_atomic_int_inc(&g_sessions_count);

	janus_mutex_lock(&sessions_mutex);
	g_hash_table_insert(sessions, handle, session);
	janus_mutex_unlock(&sessions_mutex);

	return;
}

static janus_videoroom_publisher *janus_videoroom_session_get_publisher(janus_videoroom_session *session) {
	janus_mutex_lock(&session->mutex);
	janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)session->participant;
	if(publisher)
		janus_refcount_increase(&publisher->ref);
	janus_mutex_unlock(&session->mutex);
	return publisher;
}

static janus_videoroom_publisher *janus_videoroom_session_get_publisher_nodebug(janus_videoroom_session *session) {
	janus_mutex_lock(&session->mutex);
	janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)session->participant;
	if(publisher)
		janus_refcount_increase_nodebug(&publisher->ref);
	janus_mutex_unlock(&session->mutex);
	return publisher;
}

static janus_videoroom_subscriber *janus_videoroom_session_get_subscriber(janus_videoroom_session *session) {
	janus_mutex_lock(&session->mutex);
	janus_videoroom_subscriber *subscriber = (janus_videoroom_subscriber *)session->participant;
	if(subscriber)
		janus_refcount_increase(&subscriber->ref);
	janus_mutex_unlock(&session->mutex);
	return subscriber;
}

static janus_videoroom_subscriber *janus_videoroom_session_get_subscriber_nodebug(janus_videoroom_session *session) {
	janus_mutex_lock(&session->mutex);
	janus_videoroom_subscriber *subscriber = (janus_videoroom_subscriber *)session->participant;
	if(subscriber)
		janus_refcount_increase_nodebug(&subscriber->ref);
	janus_mutex_unlock(&session->mutex);
	return subscriber;
}

static void janus_videoroom_notify_participants(janus_videoroom_publisher *participant, json_t *msg, gboolean notify_source_participant) {
	/* participant->room->mutex has to be locked. */
	if(participant->room == NULL)
		return;
	GHashTableIter iter;
	gpointer value;
	g_hash_table_iter_init(&iter, participant->room->participants);
	while (participant->room && !g_atomic_int_get(&participant->room->destroyed) && g_hash_table_iter_next(&iter, NULL, &value)) {
		janus_videoroom_publisher *p = value;
		if(p && !g_atomic_int_get(&p->destroyed) && p->session && (p != participant || notify_source_participant)) {
			JANUS_LOG(LOG_VERB, "Notifying participant %s (%s)\n", p->user_id_str, p->display ? p->display : "??");
			int ret = gateway->push_event(p->session->handle, &janus_videoroom_plugin, NULL, msg, NULL);
			JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
		}
	}
}

static void janus_videoroom_notify_about_publisher(janus_videoroom_publisher *p, gboolean update) {
	if(p == NULL)
		return;
	/* Notify all other participants that there's a new boy in town */
	json_t *list = json_array();
	json_t *pl = json_object();
	json_object_set_new(pl, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
	if(p->display)
		json_object_set_new(pl, "display", json_string(p->display));
	/* Add proper info on all the streams */
	gboolean audio_added = FALSE, video_added = FALSE;
	json_t *media = json_array();
	GList *temp = p->streams;
	while(temp) {
		janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
		json_t *info = json_object();
		json_object_set_new(info, "type", json_string(janus_videoroom_media_str(ps->type)));
		json_object_set_new(info, "mindex", json_integer(ps->mindex));
		json_object_set_new(info, "mid", json_string(ps->mid));
		if(ps->disabled) {
			json_object_set_new(info, "disabled", json_true());
		} else {
			if(ps->description)
				json_object_set_new(info, "description", json_string(ps->description));
			if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
				json_object_set_new(info, "codec", json_string(janus_audiocodec_name(ps->acodec)));
				/* FIXME For backwards compatibility, we need audio_codec in the global info */
				if(!audio_added) {
					audio_added = TRUE;
					json_object_set_new(pl, "audio_codec", json_string(janus_audiocodec_name(ps->acodec)));
				}
				if(ps->acodec == JANUS_AUDIOCODEC_OPUS) {
					if(ps->opusstereo)
						json_object_set_new(info, "stereo", json_true());
					if(ps->opusfec)
						json_object_set_new(info, "fec", json_true());
					if(ps->opusdtx)
						json_object_set_new(info, "dtx", json_true());
				}
			} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
				json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
				/* FIXME For backwards compatibility, we need video_codec in the global info */
				if(!video_added) {
					video_added = TRUE;
					json_object_set_new(pl, "video_codec", json_string(janus_videocodec_name(ps->vcodec)));
				}
				if(ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile != NULL)
					json_object_set_new(info, "h264_profile", json_string(ps->h264_profile));
				else if(ps->vcodec == JANUS_VIDEOCODEC_VP9)
					json_object_set_new(info, "vp9_profile", json_string(ps->vp9_profile));
				if(ps->muted)
					json_object_set_new(info, "moderated", json_true());
				if(ps->simulcast)
					json_object_set_new(info, "simulcast", json_true());
				if(ps->svc)
					json_object_set_new(info, "svc", json_true());
			}
		}
		json_array_append_new(media, info);
		temp = temp->next;
	}
	json_object_set_new(pl, "streams", media);
	json_array_append_new(list, pl);
	json_t *pub = json_object();
	json_object_set_new(pub, "videoroom", json_string("event"));
	json_object_set_new(pub, "room", string_ids ? json_string(p->room_id_str) : json_integer(p->room_id));
	json_object_set_new(pub, "publishers", list);
 	janus_videoroom *room = p->room;
 	if(room && !g_atomic_int_get(&room->destroyed)) {
 		janus_refcount_increase(&room->ref);
 		janus_mutex_lock(&room->mutex);
		janus_videoroom_notify_participants(p, pub, FALSE);
		janus_mutex_unlock(&room->mutex);
 		janus_refcount_decrease(&room->ref);
	}
	json_decref(pub);
	/* Also notify event handlers */
	if(notify_events && gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string(update ? "updated" : "published"));
		json_object_set_new(info, "room", string_ids ? json_string(p->room_id_str) : json_integer(p->room_id));
		json_object_set_new(info, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
		if(p->display)
				json_object_set_new(info, "display", json_string(p->display));
		json_t *media = json_array();
		GList *temp = p->streams;
		while(temp) {
			janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
			json_t *mediainfo = json_object();
			json_object_set_new(mediainfo, "type", json_string(janus_videoroom_media_str(ps->type)));
			json_object_set_new(mediainfo, "mindex", json_integer(ps->mindex));
			json_object_set_new(mediainfo, "mid", json_string(ps->mid));
			if(ps->disabled) {
				json_object_set_new(mediainfo, "disabled", json_true());
			} else {
				if(ps->description)
					json_object_set_new(mediainfo, "description", json_string(ps->description));
				if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
					json_object_set_new(mediainfo, "codec", json_string(janus_audiocodec_name(ps->acodec)));
				} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
					json_object_set_new(mediainfo, "codec", json_string(janus_videocodec_name(ps->vcodec)));
					if(ps->muted)
						json_object_set_new(mediainfo, "moderated", json_true());
					if(ps->simulcast)
						json_object_set_new(mediainfo, "simulcast", json_true());
					if(ps->svc)
						json_object_set_new(mediainfo, "svc", json_true());
				}
			}
			json_array_append_new(media, mediainfo);
			temp = temp->next;
		}
		json_object_set_new(info, "streams", media);
		gateway->notify_event(&janus_videoroom_plugin, p->session->handle, info);
	}
}

static void janus_videoroom_participant_joining(janus_videoroom_publisher *p) {
	/* we need to check if the room still exists, may have been destroyed already */
	if(p->room == NULL)
		return;
	if(!g_atomic_int_get(&p->room->destroyed) && p->room->notify_joining) {
		json_t *event = json_object();
		json_t *user = json_object();
		json_object_set_new(user, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
		if (p->display) {
			json_object_set_new(user, "display", json_string(p->display));
		}
		json_object_set_new(event, "videoroom", json_string("event"));
		json_object_set_new(event, "room", string_ids ? json_string(p->room_id_str) : json_integer(p->room_id));
		json_object_set_new(event, "joining", user);
		janus_videoroom_notify_participants(p, event, FALSE);
		/* user gets deref-ed by the owner event */
		json_decref(event);
	}
}

static void janus_videoroom_leave_or_unpublish(janus_videoroom_publisher *participant, gboolean is_leaving, gboolean kicked) {
	/* we need to check if the room still exists, may have been destroyed already */
	if(participant->room == NULL)
		return;
	janus_mutex_lock(&rooms_mutex);
	if(!g_hash_table_lookup(rooms, string_ids ? (gpointer)participant->room_id_str : (gpointer)&participant->room_id)) {
		JANUS_LOG(LOG_ERR, "No such room (%s)\n", participant->room_id_str);
		janus_mutex_unlock(&rooms_mutex);
		return;
	}
	janus_videoroom *room = participant->room;
	if(!room || g_atomic_int_get(&room->destroyed)) {
		janus_mutex_unlock(&rooms_mutex);
		return;
	}
	janus_refcount_increase(&room->ref);
	janus_mutex_unlock(&rooms_mutex);
	janus_mutex_lock(&room->mutex);
	if (!participant->room) {
		janus_mutex_unlock(&room->mutex);
		janus_refcount_decrease(&room->ref);
		return;
	}
	json_t *event = json_object();
	json_object_set_new(event, "videoroom", json_string("event"));
	json_object_set_new(event, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
	json_object_set_new(event, is_leaving ? (kicked ? "kicked" : "leaving") : "unpublished",
		string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
	janus_videoroom_notify_participants(participant, event, FALSE);
	/* Also notify event handlers */
	if(notify_events && gateway->events_is_enabled()) {
		json_t *info = json_object();
		json_object_set_new(info, "event", json_string(is_leaving ? (kicked ? "kicked" : "leaving") : "unpublished"));
		json_object_set_new(info, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
		json_object_set_new(info, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
		gateway->notify_event(&janus_videoroom_plugin, NULL, info);
	}
	if(is_leaving) {
		g_hash_table_remove(participant->room->participants,
			string_ids ? (gpointer)participant->user_id_str : (gpointer)&participant->user_id);
		g_hash_table_remove(participant->room->private_ids, GUINT_TO_POINTER(participant->pvt_id));
		g_clear_pointer(&participant->room, janus_videoroom_room_dereference);
	}
	janus_mutex_unlock(&room->mutex);
	janus_refcount_decrease(&room->ref);
	json_decref(event);
}

void janus_videoroom_destroy_session(janus_plugin_session *handle, int *error) {
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized)) {
		*error = -1;
		return;
	}
	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session) {
		janus_mutex_unlock(&sessions_mutex);
		JANUS_LOG(LOG_ERR, "No VideoRoom session associated with this handle...\n");
		*error = -2;
		return;
	}
	if(g_atomic_int_get(&session->destroyed)) {
		janus_mutex_unlock(&sessions_mutex);
		JANUS_LOG(LOG_WARN, "VideoRoom session already marked as destroyed...\n");
		return;
	}
	janus_refcount_increase(&session->ref);
	g_hash_table_remove(sessions, handle);
	janus_mutex_unlock(&sessions_mutex);
	/* Any related WebRTC PeerConnection is not available anymore either */
	janus_videoroom_hangup_media_internal(session);
	/* Cleaning up and removing the session is done in a lazy way */
	if(session->participant_type == janus_videoroom_p_type_publisher) {
		/* Get rid of publisher */
		janus_mutex_lock(&session->mutex);
		janus_videoroom_publisher *p = (janus_videoroom_publisher *)session->participant;
		if(p)
			janus_refcount_increase(&p->ref);
		janus_videoroom_publisher_print_stat(p);
		session->participant = NULL;
		janus_mutex_unlock(&session->mutex);
		if(p && p->room) {
			janus_videoroom_leave_or_unpublish(p, TRUE, FALSE);
		}
		janus_videoroom_publisher_destroy(p);
		if(p)
			janus_refcount_decrease(&p->ref);
	} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
		janus_mutex_lock(&session->mutex);
		janus_videoroom_subscriber *s = (janus_videoroom_subscriber *)session->participant;
		if(s)
			janus_refcount_increase(&s->ref);
		session->participant = NULL;
		janus_mutex_unlock(&session->mutex);
		if(s && s->room) {
			if(s->pvt_id > 0) {
				janus_mutex_lock(&s->room->mutex);
				janus_videoroom_publisher *owner = g_hash_table_lookup(s->room->private_ids, GUINT_TO_POINTER(s->pvt_id));
				if(owner != NULL) {
					janus_mutex_lock(&owner->subscribers_mutex);
					/* Note: we should refcount these subscription-publisher mappings as well */
					owner->subscriptions = g_slist_remove(owner->subscriptions, s);
					janus_mutex_unlock(&owner->subscribers_mutex);
				}
				janus_mutex_unlock(&s->room->mutex);
			}
			janus_refcount_decrease(&s->room->ref);
		}
		janus_videoroom_subscriber_destroy(s);
		if(s)
			janus_refcount_decrease(&s->ref);
	}
	janus_refcount_decrease(&session->ref);
	return;
}

json_t *janus_videoroom_query_session(janus_plugin_session *handle) {
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized)) {
		return NULL;
	}
	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session) {
		janus_mutex_unlock(&sessions_mutex);
		JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
		return NULL;
	}
	janus_refcount_increase(&session->ref);
	janus_mutex_unlock(&sessions_mutex);
	/* Show the participant/room info, if any */
	json_t *info = json_object();
	if(session->participant) {
		if(session->participant_type == janus_videoroom_p_type_none) {
			json_object_set_new(info, "type", json_string("none"));
		} else if(session->participant_type == janus_videoroom_p_type_publisher) {
			json_object_set_new(info, "type", json_string("publisher"));
			janus_videoroom_publisher *participant = janus_videoroom_session_get_publisher(session);
			if(participant && participant->room) {
				janus_videoroom *room = participant->room;
				json_object_set_new(info, "room", room ?
					(string_ids ? json_string(room->room_id_str) : json_integer(room->room_id)) : NULL);
				json_object_set_new(info, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
				json_object_set_new(info, "private_id", json_integer(participant->pvt_id));
				if(participant->display)
					json_object_set_new(info, "display", json_string(participant->display));
				/* TODO Fix the summary of viewers, since everything is stream based now */
				//~ if(participant->subscribers)
					//~ json_object_set_new(info, "viewers", json_integer(g_slist_length(participant->subscribers)));
				json_object_set_new(info, "bitrate", json_integer(participant->bitrate));
				if(participant->e2ee)
					json_object_set_new(info, "e2ee", json_true());
				json_t *media = json_array();
				GList *temp = participant->streams;
				while(temp) {
					janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
					janus_refcount_increase(&ps->ref);
					json_t *m = json_object();
					json_object_set_new(m, "type", json_string(janus_videoroom_media_str(ps->type)));
					json_object_set_new(m, "mindex", json_integer(ps->mindex));
					json_object_set_new(m, "mid", json_string(ps->mid));
					if(ps->description)
						json_object_set_new(m, "description", json_string(ps->description));
					if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
						json_object_set_new(m, "codec", json_string(janus_audiocodec_name(ps->acodec)));
					} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
						json_object_set_new(m, "codec", json_string(janus_videocodec_name(ps->vcodec)));
						if(ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile != NULL)
							json_object_set_new(m, "h264-profile", json_string(ps->h264_profile));
						if(ps->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile != NULL)
							json_object_set_new(m, "vp9-profile", json_string(ps->vp9_profile));
						if(ps->min_delay > -1 && ps->max_delay > -1) {
							json_t *pd = json_object();
							json_object_set_new(pd, "min-delay", json_integer(ps->min_delay));
							json_object_set_new(pd, "max-delay", json_integer(ps->max_delay));
							json_object_set_new(m, "playout-delay", pd);
						}
					}
					if(ps->simulcast)
						json_object_set_new(m, "simulcast", json_true());
					if(ps->svc)
						json_object_set_new(m, "vp9-svc", json_true());
					if(ps->rc && ps->rc->filename)
						json_object_set_new(m, "recording", json_string(ps->rc->filename));
					if(ps->audio_level_extmap_id > 0) {
						json_object_set_new(m, "audio-level-dBov", json_integer(ps->audio_dBov_level));
						json_object_set_new(m, "talking", ps->talking ? json_true() : json_false());
					}
					janus_mutex_lock(&ps->subscribers_mutex);
					json_object_set_new(m, "subscribers", json_integer(g_slist_length(ps->subscribers)));
					janus_mutex_unlock(&ps->subscribers_mutex);
					janus_refcount_decrease(&ps->ref);
					json_array_append_new(media, m);
					temp = temp->next;
				}
				json_object_set_new(info, "streams", media);
			}
			if(participant != NULL)
				janus_refcount_decrease(&participant->ref);
		} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
			json_object_set_new(info, "type", json_string("subscriber"));
			janus_videoroom_subscriber *participant = janus_videoroom_session_get_subscriber(session);
			if(participant && participant->room) {
				janus_videoroom *room = participant->room;
				json_object_set_new(info, "room", room ?
					(string_ids ? json_string(room->room_id_str) : json_integer(room->room_id)) : NULL);
				json_object_set_new(info, "private_id", json_integer(participant->pvt_id));
				json_object_set_new(info, "answered", g_atomic_int_get(&participant->answered) ? json_true() : json_false());
				json_object_set_new(info, "pending_offer", g_atomic_int_get(&participant->pending_offer) ? json_true() : json_false());
				json_object_set_new(info, "pending_restart", g_atomic_int_get(&participant->pending_restart) ? json_true() : json_false());
				json_object_set_new(info, "paused", participant->paused ? json_true() : json_false());
				if(participant->e2ee)
					json_object_set_new(info, "e2ee", json_true());
				janus_mutex_lock(&participant->streams_mutex);
				json_t *media = janus_videoroom_subscriber_streams_summary(participant, FALSE, NULL);
				janus_mutex_unlock(&participant->streams_mutex);
				json_object_set_new(info, "streams", media);
			}
			if(participant)
				janus_refcount_decrease(&participant->ref);
		}
	}
	json_object_set_new(info, "hangingup", json_integer(g_atomic_int_get(&session->hangingup)));
	json_object_set_new(info, "destroyed", json_integer(g_atomic_int_get(&session->destroyed)));
	janus_refcount_decrease(&session->ref);
	return info;
}

static int janus_videoroom_access_room(json_t *root, gboolean check_modify, gboolean check_join, janus_videoroom **videoroom, char *error_cause, int error_cause_size) {
	/* rooms_mutex has to be locked */
	int error_code = 0;
	json_t *room = json_object_get(root, "room");
	guint64 room_id = 0;
	char room_id_num[30], *room_id_str = NULL;
	if(!string_ids) {
		room_id = json_integer_value(room);
		g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
		room_id_str = room_id_num;
	} else {
		room_id_str = (char *)json_string_value(room);
	}
	*videoroom = g_hash_table_lookup(rooms,
		string_ids ? (gpointer)room_id_str : (gpointer)&room_id);
	if(*videoroom == NULL) {
		JANUS_LOG(LOG_ERR, "No such room (%s)\n", room_id_str);
		error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
		if(error_cause)
			g_snprintf(error_cause, error_cause_size, "No such room (%s)", room_id_str);
		return error_code;
	}
	if(g_atomic_int_get(&((*videoroom)->destroyed))) {
		JANUS_LOG(LOG_ERR, "No such room (%s)\n", room_id_str);
		error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
		if(error_cause)
			g_snprintf(error_cause, error_cause_size, "No such room (%s)", room_id_str);
		return error_code;
	}
	if(check_modify) {
		char error_cause2[100];
		JANUS_CHECK_SECRET((*videoroom)->room_secret, root, "secret", error_code, error_cause2,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
		if(error_code != 0) {
			g_strlcpy(error_cause, error_cause2, error_cause_size);
			return error_code;
		}
	}
	if(check_join) {
		char error_cause2[100];
		/* Signed tokens are enforced, so they precede any pin validation */
		if(gateway->auth_is_signed() && (*videoroom)->signed_tokens) {
			json_t *token = json_object_get(root, "token");
			char room_descriptor[100];
			g_snprintf(room_descriptor, sizeof(room_descriptor), "room=%s", room_id_str);
			if(!gateway->auth_signature_contains(&janus_videoroom_plugin, json_string_value(token), room_descriptor)) {
				error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
				if(error_cause)
					g_snprintf(error_cause, error_cause_size, "Unauthorized (wrong token)");
				return error_code;
			}
		}
		JANUS_CHECK_SECRET((*videoroom)->room_pin, root, "pin", error_code, error_cause2,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
		if(error_code != 0) {
			g_strlcpy(error_cause, error_cause2, error_cause_size);
			return error_code;
		}
	}
	return 0;
}

/* Helper method to process synchronous requests */
static json_t *janus_videoroom_process_synchronous_request(janus_videoroom_session *session, json_t *message) {
	json_t *request = json_object_get(message, "request");
	const char *request_text = json_string_value(request);

	/* Parse the message */
	int error_code = 0;
	char error_cause[512];
	json_t *root = message;
	json_t *response = NULL;

	if(!strcasecmp(request_text, "create")) {
		/* Create a new VideoRoom */
		JANUS_LOG(LOG_VERB, "Creating a new VideoRoom room\n");
		JANUS_VALIDATE_JSON_OBJECT(root, create_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, roomopt_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstropt_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		if(admin_key != NULL) {
			/* An admin key was specified: make sure it was provided, and that it's valid */
			JANUS_VALIDATE_JSON_OBJECT(root, adminkey_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			if(error_code != 0)
				goto prepare_response;
			JANUS_CHECK_SECRET(admin_key, root, "admin_key", error_code, error_cause,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
			if(error_code != 0)
				goto prepare_response;
		}
		json_t *desc = json_object_get(root, "description");
		json_t *is_private = json_object_get(root, "is_private");
		json_t *req_pvtid = json_object_get(root, "require_pvtid");
		json_t *signed_tokens = json_object_get(root, "signed_tokens");
		json_t *req_e2ee = json_object_get(root, "require_e2ee");
		json_t *dummy_pub = json_object_get(root, "dummy_publisher");
		json_t *dummy_str = json_object_get(root, "dummy_streams");
		json_t *secret = json_object_get(root, "secret");
		json_t *pin = json_object_get(root, "pin");
		json_t *bitrate = json_object_get(root, "bitrate");
		json_t *bitrate_cap = json_object_get(root, "bitrate_cap");
		json_t *fir_freq = json_object_get(root, "fir_freq");
		json_t *publishers = json_object_get(root, "publishers");
		json_t *allowed = json_object_get(root, "allowed");
		json_t *audiocodec = json_object_get(root, "audiocodec");
		if(audiocodec) {
			const char *audiocodec_value = json_string_value(audiocodec);
			gchar **list = g_strsplit(audiocodec_value, ",", 6);
			gchar *codec = list[0];
			if(codec != NULL) {
				int i=0;
				while(codec != NULL) {
					if(i == 5) {
						break;
					}
					if(strlen(codec) == 0 || JANUS_AUDIOCODEC_NONE == janus_audiocodec_from_name(codec)) {
						JANUS_LOG(LOG_ERR, "Invalid element (audiocodec can only be or contain opus, isac32, isac16, pcmu, pcma or g722)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid element (audiocodec can only be or contain opus, isac32, isac16, pcmu, pcma or g722)");
						goto prepare_response;
					}
					i++;
					codec = list[i];
				}
			}
			g_clear_pointer(&list, g_strfreev);
		}
		json_t *videocodec = json_object_get(root, "videocodec");
		if(videocodec) {
			const char *videocodec_value = json_string_value(videocodec);
			gchar **list = g_strsplit(videocodec_value, ",", 6);
			gchar *codec = list[0];
			if(codec != NULL) {
				int i=0;
				while(codec != NULL) {
					if(i == 5) {
						break;
					}
					if(strlen(codec) == 0 || JANUS_VIDEOCODEC_NONE == janus_videocodec_from_name(codec)) {
						JANUS_LOG(LOG_ERR, "Invalid element (videocodec can only be or contain vp8, vp9, h264, av1 or h265)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid element (videocodec can only be or contain vp8, vp9, av1, h264 or h265)");
						goto prepare_response;
					}
					i++;
					codec = list[i];
				}
			}
			g_clear_pointer(&list, g_strfreev);
		}
		json_t *vp9profile = json_object_get(root, "vp9_profile");
		json_t *h264profile = json_object_get(root, "h264_profile");
		json_t *fec = json_object_get(root, "opus_fec");
		json_t *dtx = json_object_get(root, "opus_dtx");
		json_t *svc = json_object_get(root, "video_svc");
		json_t *audiolevel_ext = json_object_get(root, "audiolevel_ext");
		json_t *audiolevel_event = json_object_get(root, "audiolevel_event");
		json_t *audio_active_packets = json_object_get(root, "audio_active_packets");
		json_t *audio_level_average = json_object_get(root, "audio_level_average");
		json_t *videoorient_ext = json_object_get(root, "videoorient_ext");
		json_t *playoutdelay_ext = json_object_get(root, "playoutdelay_ext");
		json_t *transport_wide_cc_ext = json_object_get(root, "transport_wide_cc_ext");
		json_t *notify_joining = json_object_get(root, "notify_joining");
		json_t *record = json_object_get(root, "record");
		json_t *rec_dir = json_object_get(root, "rec_dir");
		json_t *lock_record = json_object_get(root, "lock_record");
		json_t *permanent = json_object_get(root, "permanent");
		if(allowed) {
			/* Make sure the "allowed" array only contains strings */
			gboolean ok = TRUE;
			if(json_array_size(allowed) > 0) {
				size_t i = 0;
				for(i=0; i<json_array_size(allowed); i++) {
					json_t *a = json_array_get(allowed, i);
					if(!a || !json_is_string(a)) {
						ok = FALSE;
						break;
					}
				}
			}
			if(!ok) {
				JANUS_LOG(LOG_ERR, "Invalid element in the allowed array (not a string)\n");
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
				g_snprintf(error_cause, 512, "Invalid element in the allowed array (not a string)");
				goto prepare_response;
			}
		}
		gboolean save = permanent ? json_is_true(permanent) : FALSE;
		if(save && config == NULL) {
			JANUS_LOG(LOG_ERR, "No configuration file, can't create permanent room\n");
			error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
			g_snprintf(error_cause, 512, "No configuration file, can't create permanent room");
			goto prepare_response;
		}
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		json_t *room = json_object_get(root, "room");
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		if(room_id == 0 && room_id_str == NULL) {
			JANUS_LOG(LOG_WARN, "Desired room ID is empty, which is not allowed... picking random ID instead\n");
		}
		janus_mutex_lock(&rooms_mutex);
		if(room_id > 0 || room_id_str != NULL) {
			/* Let's make sure the room doesn't exist already */
			if(g_hash_table_lookup(rooms, string_ids ? (gpointer)room_id_str : (gpointer)&room_id) != NULL) {
				/* It does... */
				janus_mutex_unlock(&rooms_mutex);
				error_code = JANUS_VIDEOROOM_ERROR_ROOM_EXISTS;
				JANUS_LOG(LOG_ERR, "Room %s already exists!\n", room_id_str);
				g_snprintf(error_cause, 512, "Room %s already exists", room_id_str);
				goto prepare_response;
			}
		}
		/* Create the room */
		janus_videoroom *videoroom = g_malloc0(sizeof(janus_videoroom));
		/* Generate a random ID */
		gboolean room_id_allocated = FALSE;
		if(!string_ids && room_id == 0) {
			while(room_id == 0) {
				room_id = janus_random_uint64();
				if(g_hash_table_lookup(rooms, &room_id) != NULL) {
					/* Room ID already taken, try another one */
					room_id = 0;
				}
			}
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else if(string_ids && room_id_str == NULL) {
			while(room_id_str == NULL) {
				room_id_str = janus_random_uuid();
				if(g_hash_table_lookup(rooms, room_id_str) != NULL) {
					/* Room ID already taken, try another one */
					g_clear_pointer(&room_id_str, g_free);
				}
			}
			room_id_allocated = TRUE;
		}
		videoroom->room_id = room_id;
		videoroom->room_id_str = room_id_str ? g_strdup(room_id_str) : NULL;
		if(room_id_allocated)
			g_free(room_id_str);
		char *description = NULL;
		if(desc != NULL && strlen(json_string_value(desc)) > 0) {
			description = g_strdup(json_string_value(desc));
		} else {
			char roomname[255];
			g_snprintf(roomname, 255, "Room %s", videoroom->room_id_str);
			description = g_strdup(roomname);
		}
		videoroom->room_name = description;
		videoroom->is_private = is_private ? json_is_true(is_private) : FALSE;
		videoroom->require_pvtid = req_pvtid ? json_is_true(req_pvtid) : FALSE;
		if(signed_tokens && json_is_true(signed_tokens)) {
			if(!gateway->auth_is_signed()) {
				JANUS_LOG(LOG_WARN, "Can't enforce signed tokens for this room, signed-mode not in use in the core\n");
			} else {
				videoroom->signed_tokens = TRUE;
			}
		}
		videoroom->require_e2ee = req_e2ee ? json_is_true(req_e2ee) : FALSE;
		if(secret)
			videoroom->room_secret = g_strdup(json_string_value(secret));
		if(pin)
			videoroom->room_pin = g_strdup(json_string_value(pin));
		videoroom->max_publishers = 3;	/* FIXME How should we choose a default? */
		if(publishers)
			videoroom->max_publishers = json_integer_value(publishers);
		if(videoroom->max_publishers < 0)
			videoroom->max_publishers = 3;	/* FIXME How should we choose a default? */
		videoroom->min_audio_mix_participants = MIN_MIX_PARTICIPANTS;
		videoroom->audio_mix_active = FALSE;
		videoroom->bitrate = 0;
		if(bitrate)
			videoroom->bitrate = json_integer_value(bitrate);
		if(videoroom->bitrate > 0 && videoroom->bitrate < 64000)
			videoroom->bitrate = 64000;	/* Don't go below 64k */
		videoroom->bitrate_cap = bitrate_cap ? json_is_true(bitrate_cap) : FALSE;
		videoroom->fir_freq = 0;
		if(fir_freq)
			videoroom->fir_freq = json_integer_value(fir_freq);
		/* By default, we force Opus as the only audio codec */
		videoroom->acodec[0] = JANUS_AUDIOCODEC_OPUS;
		videoroom->acodec[1] = JANUS_AUDIOCODEC_NONE;
		videoroom->acodec[2] = JANUS_AUDIOCODEC_NONE;
		videoroom->acodec[3] = JANUS_AUDIOCODEC_NONE;
		videoroom->acodec[4] = JANUS_AUDIOCODEC_NONE;
		/* Check if we're forcing a different single codec, or allowing more than one */
		if(audiocodec) {
			const char *audiocodec_value = json_string_value(audiocodec);
			gchar **list = g_strsplit(audiocodec_value, ",", 6);
			gchar *codec = list[0];
			if(codec != NULL) {
				int i=0;
				while(codec != NULL) {
					if(i == 5) {
						JANUS_LOG(LOG_WARN, "Ignoring extra audio codecs: %s\n", codec);
						break;
					}
					if(strlen(codec) > 0)
						videoroom->acodec[i] = janus_audiocodec_from_name(codec);
					i++;
					codec = list[i];
				}
			}
			g_clear_pointer(&list, g_strfreev);
		}
		/* By default, we force VP8 as the only video codec */
		videoroom->vcodec[0] = JANUS_VIDEOCODEC_VP8;
		videoroom->vcodec[1] = JANUS_VIDEOCODEC_NONE;
		videoroom->vcodec[2] = JANUS_VIDEOCODEC_NONE;
		videoroom->vcodec[3] = JANUS_VIDEOCODEC_NONE;
		videoroom->vcodec[4] = JANUS_VIDEOCODEC_NONE;
		/* Check if we're forcing a different single codec, or allowing more than one */
		if(videocodec) {
			const char *videocodec_value = json_string_value(videocodec);
			gchar **list = g_strsplit(videocodec_value, ",", 6);
			gchar *codec = list[0];
			if(codec != NULL) {
				int i=0;
				while(codec != NULL) {
					if(i == 5) {
						JANUS_LOG(LOG_WARN, "Ignoring extra video codecs: %s\n", codec);
						break;
					}
					if(strlen(codec) > 0)
						videoroom->vcodec[i] = janus_videocodec_from_name(codec);
					i++;
					codec = list[i];
				}
			}
			g_clear_pointer(&list, g_strfreev);
		}
		const char *vp9_profile = json_string_value(vp9profile);
		if(vp9_profile && (videoroom->vcodec[0] == JANUS_VIDEOCODEC_VP9 ||
				videoroom->vcodec[1] == JANUS_VIDEOCODEC_VP9 ||
				videoroom->vcodec[2] == JANUS_VIDEOCODEC_VP9 ||
				videoroom->vcodec[3] == JANUS_VIDEOCODEC_VP9 ||
				videoroom->vcodec[4] == JANUS_VIDEOCODEC_VP9)) {
			videoroom->vp9_profile = g_strdup(vp9_profile);
		}
		const char *h264_profile = json_string_value(h264profile);
		if(h264_profile && (videoroom->vcodec[0] == JANUS_VIDEOCODEC_H264 ||
				videoroom->vcodec[1] == JANUS_VIDEOCODEC_H264 ||
				videoroom->vcodec[2] == JANUS_VIDEOCODEC_H264 ||
				videoroom->vcodec[3] == JANUS_VIDEOCODEC_H264 ||
				videoroom->vcodec[4] == JANUS_VIDEOCODEC_H264)) {
			videoroom->h264_profile = g_strdup(h264_profile);
		}
		videoroom->do_opusfec = TRUE;
		if(fec) {
			videoroom->do_opusfec = json_is_true(fec);
			if(videoroom->acodec[0] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[1] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[2] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[3] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[4] != JANUS_AUDIOCODEC_OPUS) {
				videoroom->do_opusfec = FALSE;
				JANUS_LOG(LOG_WARN, "Inband FEC is only supported for rooms that allow Opus: disabling it...\n");
			}
		}
		if(dtx) {
			videoroom->do_opusdtx = json_is_true(dtx);
			if(videoroom->acodec[0] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[1] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[2] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[3] != JANUS_AUDIOCODEC_OPUS &&
					videoroom->acodec[4] != JANUS_AUDIOCODEC_OPUS) {
				videoroom->do_opusdtx = FALSE;
				JANUS_LOG(LOG_WARN, "DTX is only supported for rooms that allow Opus: disabling it...\n");
			}
		}
		if(svc && json_is_true(svc)) {
			if(videoroom->vcodec[0] == JANUS_VIDEOCODEC_VP9 &&
					videoroom->vcodec[1] == JANUS_VIDEOCODEC_NONE &&
					videoroom->vcodec[2] == JANUS_VIDEOCODEC_NONE &&
					videoroom->vcodec[3] == JANUS_VIDEOCODEC_NONE &&
					videoroom->vcodec[4] == JANUS_VIDEOCODEC_NONE) {
				videoroom->do_svc = TRUE;
			} else {
				JANUS_LOG(LOG_WARN, "SVC is only supported, in an experimental way, for VP9 only rooms: disabling it...\n");
			}
		}
		videoroom->audiolevel_ext = audiolevel_ext ? json_is_true(audiolevel_ext) : TRUE;
		videoroom->audiolevel_event = audiolevel_event ? json_is_true(audiolevel_event) : FALSE;
		if(videoroom->audiolevel_event) {
			videoroom->audio_active_packets = 100;
			if(json_integer_value(audio_active_packets) > 0) {
				videoroom->audio_active_packets = json_integer_value(audio_active_packets);
			} else {
				JANUS_LOG(LOG_WARN, "Invalid audio_active_packets value provided, using default: %d\n", videoroom->audio_active_packets);
			}
			videoroom->audio_level_average = 25;
			if(json_integer_value(audio_level_average) > 0) {
				videoroom->audio_level_average = json_integer_value(audio_level_average);
			} else {
				JANUS_LOG(LOG_WARN, "Invalid audio_level_average value provided, using default: %d\n", videoroom->audio_level_average);
			}
		}
		videoroom->videoorient_ext = videoorient_ext ? json_is_true(videoorient_ext) : TRUE;
		videoroom->playoutdelay_ext = playoutdelay_ext ? json_is_true(playoutdelay_ext) : TRUE;
		videoroom->transport_wide_cc_ext = transport_wide_cc_ext ? json_is_true(transport_wide_cc_ext) : TRUE;
		/* By default, the VideoRoom plugin does not notify about participants simply joining the room.
		   It only notifies when the participant actually starts publishing media. */
		videoroom->notify_joining = notify_joining ? json_is_true(notify_joining) : FALSE;
		if(record) {
			videoroom->record = json_is_true(record);
		}
		if(rec_dir) {
			videoroom->rec_dir = g_strdup(json_string_value(rec_dir));
		}
		if(lock_record) {
			videoroom->lock_record = json_is_true(lock_record);
		}
		g_atomic_int_set(&videoroom->destroyed, 0);
		janus_mutex_init(&videoroom->mutex);
		janus_refcount_init(&videoroom->ref, janus_videoroom_room_free);
		print_resource_usage("before room ctor");
		g_atomic_int_inc(&g_rooms_count);
		videoroom->participants = g_hash_table_new_full(string_ids ? g_str_hash : g_int64_hash, string_ids ? g_str_equal : g_int64_equal,
			(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_publisher_dereference);
		videoroom->private_ids = g_hash_table_new(NULL, NULL);
		videoroom->allowed = g_hash_table_new_full(g_str_hash, g_str_equal, (GDestroyNotify)g_free, NULL);
		if(allowed != NULL) {
			/* Populate the "allowed" list as an ACL for people trying to join */
			if(json_array_size(allowed) > 0) {
				size_t i = 0;
				for(i=0; i<json_array_size(allowed); i++) {
					const char *token = json_string_value(json_array_get(allowed, i));
					if(!g_hash_table_lookup(videoroom->allowed, token))
						g_hash_table_insert(videoroom->allowed, g_strdup(token), GINT_TO_POINTER(TRUE));
				}
			}
			videoroom->check_allowed = TRUE;
		}

		(void)dummy_str;
		(void)dummy_pub;
		// override API: create dummy publisher for delivering mixed audio signal
		videoroom->dummy_publisher = TRUE;
		GHashTable *dummy_streams = g_hash_table_new_full(g_str_hash, g_str_equal, (GDestroyNotify)g_free, (GDestroyNotify)g_free);
		g_hash_table_insert(dummy_streams, g_strdup("opus"), g_strdup("minptime=10;useinbandfec=1"));
		janus_videoroom_create_dummy_publisher(videoroom, dummy_streams, g_audio_mix_dummy_publisher_name, 111111);
		if(dummy_streams != NULL)
			g_hash_table_destroy(dummy_streams);

		char tname[16];
		g_snprintf(tname, sizeof(tname), "audiomix%"SCNu64, room_id);
		GError *error = NULL;
		janus_refcount_increase(&videoroom->ref);
		videoroom->thread = g_thread_try_new(tname, &speech_mix_thread, videoroom, &error);
		if (error != NULL) {
			JANUS_LOG(LOG_ERR, "Got error %d (%s) trying to launch the mixer thread...\n", error->code, error->message ? error->message : "??");
			error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
			g_snprintf(error_cause, 512, "Got error %d (%s) trying to launch the mixer thread", error->code, error->message ? error->message : "??");
			janus_refcount_decrease(&videoroom->ref);
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}

		/* Compute a list of the supported codecs for the summary */
		char audio_codecs[100], video_codecs[100];
		janus_videoroom_codecstr(videoroom, audio_codecs, video_codecs, sizeof(audio_codecs), "|");
		JANUS_LOG(LOG_VERB, "Created VideoRoom: %s (%s, %s, %s/%s codecs, secret: %s, pin: %s, pvtid: %s)\n",
			videoroom->room_id_str, videoroom->room_name,
			videoroom->is_private ? "private" : "public",
			audio_codecs, video_codecs,
			videoroom->room_secret ? videoroom->room_secret : "no secret",
			videoroom->room_pin ? videoroom->room_pin : "no pin",
			videoroom->require_pvtid ? "required" : "optional");
		if(videoroom->record) {
			JANUS_LOG(LOG_VERB, "  -- Room is going to be recorded in %s\n", videoroom->rec_dir ? videoroom->rec_dir : "the current folder");
		}
		if(videoroom->require_e2ee) {
			JANUS_LOG(LOG_VERB, "  -- All publishers MUST use end-to-end encryption\n");
		}
		if(videoroom->dummy_publisher) {
			JANUS_LOG(LOG_VERB, "  -- The room is going to have a dummy publisher for placeholder subscriptions\n");
		}
		if(save) {
			/* This room is permanent: save to the configuration file too
			 * FIXME: We should check if anything fails... */
			JANUS_LOG(LOG_VERB, "Saving room %s permanently in config file\n", videoroom->room_id_str);
			janus_mutex_lock(&config_mutex);
			char cat[BUFSIZ], value[BUFSIZ];
			/* The room ID is the category (prefixed by "room-") */
			g_snprintf(cat, BUFSIZ, "room-%s", videoroom->room_id_str);
			janus_config_category *c = janus_config_get_create(config, NULL, janus_config_type_category, cat);
			/* Now for the values */
			janus_config_add(config, c, janus_config_item_create("description", videoroom->room_name));
			if(videoroom->is_private)
				janus_config_add(config, c, janus_config_item_create("is_private", "yes"));
			if(videoroom->require_pvtid)
				janus_config_add(config, c, janus_config_item_create("require_pvtid", "yes"));
			if(videoroom->signed_tokens)
				janus_config_add(config, c, janus_config_item_create("signed_tokens", "yes"));
			if(videoroom->require_e2ee)
				janus_config_add(config, c, janus_config_item_create("require_e2ee", "yes"));
			if(videoroom->dummy_publisher)
				janus_config_add(config, c, janus_config_item_create("dummy_publisher", "yes"));
			g_snprintf(value, BUFSIZ, "%"SCNu32, videoroom->bitrate);
			janus_config_add(config, c, janus_config_item_create("bitrate", value));
			if(videoroom->bitrate_cap)
				janus_config_add(config, c, janus_config_item_create("bitrate_cap", "yes"));
			g_snprintf(value, BUFSIZ, "%d", videoroom->max_publishers);
			janus_config_add(config, c, janus_config_item_create("publishers", value));
			if(videoroom->fir_freq) {
				g_snprintf(value, BUFSIZ, "%"SCNu16, videoroom->fir_freq);
				janus_config_add(config, c, janus_config_item_create("fir_freq", value));
			}
			char video_codecs[100];
			char audio_codecs[100];
			janus_videoroom_codecstr(videoroom, audio_codecs, video_codecs, sizeof(audio_codecs), ",");
			janus_config_add(config, c, janus_config_item_create("audiocodec", audio_codecs));
			janus_config_add(config, c, janus_config_item_create("videocodec", video_codecs));
			if(videoroom->vp9_profile)
				janus_config_add(config, c, janus_config_item_create("vp9_profile", videoroom->vp9_profile));
			if(videoroom->h264_profile)
				janus_config_add(config, c, janus_config_item_create("h264_profile", videoroom->h264_profile));
			if(videoroom->do_opusfec)
				janus_config_add(config, c, janus_config_item_create("opus_fec", "yes"));
			if(videoroom->do_opusdtx)
				janus_config_add(config, c, janus_config_item_create("opus_dtx", "yes"));
			if(videoroom->do_svc)
				janus_config_add(config, c, janus_config_item_create("video_svc", "yes"));
			if(videoroom->room_secret)
				janus_config_add(config, c, janus_config_item_create("secret", videoroom->room_secret));
			if(videoroom->room_pin)
				janus_config_add(config, c, janus_config_item_create("pin", videoroom->room_pin));
			if(videoroom->audiolevel_ext) {
				janus_config_add(config, c, janus_config_item_create("audiolevel_ext", "yes"));
				if(videoroom->audiolevel_event)
					janus_config_add(config, c, janus_config_item_create("audiolevel_event", "yes"));
				if(videoroom->audio_active_packets > 0) {
					g_snprintf(value, BUFSIZ, "%d", videoroom->audio_active_packets);
					janus_config_add(config, c, janus_config_item_create("audio_active_packets", value));
				}
				if(videoroom->audio_level_average > 0) {
					g_snprintf(value, BUFSIZ, "%d", videoroom->audio_level_average);
					janus_config_add(config, c, janus_config_item_create("audio_level_average", value));
				}
			} else {
				janus_config_add(config, c, janus_config_item_create("audiolevel_ext", "no"));
			}
			janus_config_add(config, c, janus_config_item_create("videoorient_ext", videoroom->videoorient_ext ? "yes" : "no"));
			janus_config_add(config, c, janus_config_item_create("playoutdelay_ext", videoroom->playoutdelay_ext ? "yes" : "no"));
			janus_config_add(config, c, janus_config_item_create("transport_wide_cc_ext", videoroom->transport_wide_cc_ext ? "yes" : "no"));
			if(videoroom->notify_joining)
				janus_config_add(config, c, janus_config_item_create("notify_joining", "yes"));
			if(videoroom->record)
				janus_config_add(config, c, janus_config_item_create("record", "yes"));
			if(videoroom->rec_dir)
				janus_config_add(config, c, janus_config_item_create("rec_dir", videoroom->rec_dir));
			if(videoroom->lock_record)
				janus_config_add(config, c, janus_config_item_create("lock_record", "yes"));
			/* Save modified configuration */
			if(janus_config_save(config, config_folder, JANUS_VIDEOROOM_PACKAGE) < 0)
				save = FALSE;	/* This will notify the user the room is not permanent */
			janus_mutex_unlock(&config_mutex);
		}

		g_hash_table_insert(rooms,
			string_ids ? (gpointer)g_strdup(videoroom->room_id_str) : (gpointer)janus_uint64_dup(videoroom->room_id),
			videoroom);
		/* Show updated rooms list */
		GHashTableIter iter;
		gpointer value;
		g_hash_table_iter_init(&iter, rooms);
		while (g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom *vr = value;
			JANUS_LOG(LOG_VERB, "  ::: [%s][%s] %"SCNu32", max %d publishers, FIR frequency of %d seconds\n",
				vr->room_id_str, vr->room_name, vr->bitrate, vr->max_publishers, vr->fir_freq);
		}
		janus_mutex_unlock(&rooms_mutex);
		/* Send info back */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("created"));
		json_object_set_new(response, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
		json_object_set_new(response, "permanent", save ? json_true() : json_false());
		json_object_set_new(response, "multistream", json_true());
		/* Also notify event handlers */
		if(notify_events && gateway->events_is_enabled()) {
			json_t *info = json_object();
			json_object_set_new(info, "event", json_string("created"));
			json_object_set_new(info, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
			gateway->notify_event(&janus_videoroom_plugin, session ? session->handle : NULL, info);
		}
		goto prepare_response;
	} else if(!strcasecmp(request_text, "edit")) {
		/* Edit the properties for an existing VideoRoom */
		JANUS_LOG(LOG_VERB, "Attempt to edit the properties of an existing VideoRoom room\n");
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, edit_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		/* We only allow for a limited set of properties to be edited */
		json_t *desc = json_object_get(root, "new_description");
		json_t *is_private = json_object_get(root, "new_is_private");
		json_t *req_pvtid = json_object_get(root, "new_require_pvtid");
		json_t *secret = json_object_get(root, "new_secret");
		json_t *pin = json_object_get(root, "new_pin");
		json_t *bitrate = json_object_get(root, "new_bitrate");
		json_t *fir_freq = json_object_get(root, "new_fir_freq");
		json_t *publishers = json_object_get(root, "new_publishers");
		json_t *lock_record = json_object_get(root, "new_lock_record");
		json_t *rec_dir = json_object_get(root, "new_rec_dir");
		json_t *permanent = json_object_get(root, "permanent");
		gboolean save = permanent ? json_is_true(permanent) : FALSE;
		if(save && config == NULL) {
			JANUS_LOG(LOG_ERR, "No configuration file, can't edit room permanently\n");
			error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
			g_snprintf(error_cause, 512, "No configuration file, can't edit room permanently");
			goto prepare_response;
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		/* Edit the room properties that were provided */
		if(desc != NULL && strlen(json_string_value(desc)) > 0) {
			char *old_description = videoroom->room_name;
			char *new_description = g_strdup(json_string_value(desc));
			videoroom->room_name = new_description;
			g_free(old_description);
		}
		if(is_private)
			videoroom->is_private = json_is_true(is_private);
		if(req_pvtid)
			videoroom->require_pvtid = json_is_true(req_pvtid);
		if(publishers)
			videoroom->max_publishers = json_integer_value(publishers);
		if(bitrate) {
			videoroom->bitrate = json_integer_value(bitrate);
			if(videoroom->bitrate > 0 && videoroom->bitrate < 64000)
				videoroom->bitrate = 64000;	/* Don't go below 64k */
		}
		if(fir_freq)
			videoroom->fir_freq = json_integer_value(fir_freq);
		if(secret && strlen(json_string_value(secret)) > 0) {
			char *old_secret = videoroom->room_secret;
			char *new_secret = g_strdup(json_string_value(secret));
			videoroom->room_secret = new_secret;
			g_free(old_secret);
		}
		if(pin && strlen(json_string_value(pin)) > 0) {
			char *old_pin = videoroom->room_pin;
			char *new_pin = g_strdup(json_string_value(pin));
			videoroom->room_pin = new_pin;
			g_free(old_pin);
		}
		if(lock_record)
			videoroom->lock_record = json_is_true(lock_record);
		if(rec_dir) {
			char *old_rec_dir = videoroom->rec_dir;
			char *new_rec_dir = g_strdup(json_string_value(rec_dir));
			videoroom->rec_dir = new_rec_dir;
			g_free(old_rec_dir);
		}
		if(save) {
			/* This room is permanent: save to the configuration file too
			 * FIXME: We should check if anything fails... */
			JANUS_LOG(LOG_VERB, "Modifying room %s permanently in config file\n", videoroom->room_id_str);
			janus_mutex_lock(&config_mutex);
			char cat[BUFSIZ], value[BUFSIZ];
			/* The room ID is the category (prefixed by "room-") */
			g_snprintf(cat, BUFSIZ, "room-%s", videoroom->room_id_str);
			/* Remove the old category first */
			janus_config_remove(config, NULL, cat);
			/* Now write the room details again */
			janus_config_category *c = janus_config_get_create(config, NULL, janus_config_type_category, cat);
			janus_config_add(config, c, janus_config_item_create("description", videoroom->room_name));
			if(videoroom->is_private)
				janus_config_add(config, c, janus_config_item_create("is_private", "yes"));
			if(videoroom->require_pvtid)
				janus_config_add(config, c, janus_config_item_create("require_pvtid", "yes"));
			if(videoroom->signed_tokens)
				janus_config_add(config, c, janus_config_item_create("signed_tokens", "yes"));
			if(videoroom->require_e2ee)
				janus_config_add(config, c, janus_config_item_create("require_e2ee", "yes"));
			if(videoroom->dummy_publisher)
				janus_config_add(config, c, janus_config_item_create("dummy_publisher", "yes"));
			g_snprintf(value, BUFSIZ, "%"SCNu32, videoroom->bitrate);
			janus_config_add(config, c, janus_config_item_create("bitrate", value));
			if(videoroom->bitrate_cap)
				janus_config_add(config, c, janus_config_item_create("bitrate_cap", "yes"));
			g_snprintf(value, BUFSIZ, "%d", videoroom->max_publishers);
			janus_config_add(config, c, janus_config_item_create("publishers", value));
			if(videoroom->fir_freq) {
				g_snprintf(value, BUFSIZ, "%"SCNu16, videoroom->fir_freq);
				janus_config_add(config, c, janus_config_item_create("fir_freq", value));
			}
			char audio_codecs[100];
			char video_codecs[100];
			janus_videoroom_codecstr(videoroom, audio_codecs, video_codecs, sizeof(audio_codecs), ",");
			janus_config_add(config, c, janus_config_item_create("audiocodec", audio_codecs));
			janus_config_add(config, c, janus_config_item_create("videocodec", video_codecs));
			if(videoroom->vp9_profile)
				janus_config_add(config, c, janus_config_item_create("vp9_profile", videoroom->vp9_profile));
			if(videoroom->h264_profile)
				janus_config_add(config, c, janus_config_item_create("h264_profile", videoroom->h264_profile));
			if(videoroom->do_opusfec)
				janus_config_add(config, c, janus_config_item_create("opus_fec", "yes"));
			if(videoroom->do_opusdtx)
				janus_config_add(config, c, janus_config_item_create("opus_dtx", "yes"));
			if(videoroom->do_svc)
				janus_config_add(config, c, janus_config_item_create("video_svc", "yes"));
			if(videoroom->room_secret)
				janus_config_add(config, c, janus_config_item_create("secret", videoroom->room_secret));
			if(videoroom->room_pin)
				janus_config_add(config, c, janus_config_item_create("pin", videoroom->room_pin));
			if(videoroom->audiolevel_ext) {
				janus_config_add(config, c, janus_config_item_create("audiolevel_ext", "yes"));
				if(videoroom->audiolevel_event)
					janus_config_add(config, c, janus_config_item_create("audiolevel_event", "yes"));
				if(videoroom->audio_active_packets > 0) {
					g_snprintf(value, BUFSIZ, "%d", videoroom->audio_active_packets);
					janus_config_add(config, c, janus_config_item_create("audio_active_packets", value));
				}
				if(videoroom->audio_level_average > 0) {
					g_snprintf(value, BUFSIZ, "%d", videoroom->audio_level_average);
					janus_config_add(config, c, janus_config_item_create("audio_level_average", value));
				}
			} else {
				janus_config_add(config, c, janus_config_item_create("audiolevel_ext", "no"));
			}
			janus_config_add(config, c, janus_config_item_create("videoorient_ext", videoroom->videoorient_ext ? "yes" : "no"));
			janus_config_add(config, c, janus_config_item_create("playoutdelay_ext", videoroom->playoutdelay_ext ? "yes" : "no"));
			janus_config_add(config, c, janus_config_item_create("transport_wide_cc_ext", videoroom->transport_wide_cc_ext ? "yes" : "no"));
			if(videoroom->notify_joining)
				janus_config_add(config, c, janus_config_item_create("notify_joining", "yes"));
			if(videoroom->record)
				janus_config_add(config, c, janus_config_item_create("record", "yes"));
			if(videoroom->rec_dir)
				janus_config_add(config, c, janus_config_item_create("rec_dir", videoroom->rec_dir));
			if(videoroom->lock_record)
				janus_config_add(config, c, janus_config_item_create("lock_record", "yes"));
			/* Save modified configuration */
			if(janus_config_save(config, config_folder, JANUS_VIDEOROOM_PACKAGE) < 0)
				save = FALSE;	/* This will notify the user the room changes are not permanent */
			janus_mutex_unlock(&config_mutex);
		}
		janus_mutex_unlock(&rooms_mutex);
		/* Send info back */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("edited"));
		json_object_set_new(response, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
		json_object_set_new(response, "permanent", save ? json_true() : json_false());
		/* Also notify event handlers */
		if(notify_events && gateway->events_is_enabled()) {
			json_t *info = json_object();
			json_object_set_new(info, "event", json_string("edited"));
			json_object_set_new(info, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
			gateway->notify_event(&janus_videoroom_plugin, session ? session->handle : NULL, info);
		}
		goto prepare_response;
	} else if(!strcasecmp(request_text, "destroy")) {
		JANUS_LOG(LOG_VERB, "Attempt to destroy an existing VideoRoom room\n");
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, destroy_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		json_t *permanent = json_object_get(root, "permanent");
		gboolean save = permanent ? json_is_true(permanent) : FALSE;
		if(save && config == NULL) {
			JANUS_LOG(LOG_ERR, "No configuration file, can't destroy room permanently\n");
			error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
			g_snprintf(error_cause, 512, "No configuration file, can't destroy room permanently");
			goto prepare_response;
		}
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		/* Remove room, but add a reference until we're done */
		janus_refcount_increase(&videoroom->ref);

		GHashTableIter iter;
		gpointer value;

		janus_videoroom_publisher *dummy_publisher = NULL;
		janus_mutex_lock(&videoroom->mutex);
		g_hash_table_iter_init(&iter, videoroom->participants);
		while(g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom_publisher *p = value;
			if(p && !g_atomic_int_get(&p->destroyed) && p->session && p->room && !strcmp(g_audio_mix_dummy_publisher_name, p->display)) {
				dummy_publisher = p;
				break;
			}
		}
		// Do not increase dummy_publisher refcount here, since the room 'owns' it, and it's not accessible from outside.
		janus_mutex_unlock(&videoroom->mutex);
		if(dummy_publisher) {
			janus_mutex_unlock(&rooms_mutex);
			janus_videoroom_leave_or_unpublish(dummy_publisher, TRUE, FALSE);
			janus_videoroom_publisher_destroy(dummy_publisher);
			// dummy_publisher holds 3 references to it's publisher_stream, which holds a reference to the owner, making circular reference
			// Unref the publisher to break the circle
			// Note: for any other publisher the stream dies first, as soon as session_destroy called, but dummy publisher have dummy session
			janus_videoroom_publisher_dereference(dummy_publisher);
			janus_mutex_lock(&rooms_mutex);
		}

		g_hash_table_remove(rooms, string_ids ? (gpointer)room_id_str : (gpointer)&room_id);
		/* Notify all participants that the fun is over, and that they'll be kicked */
		JANUS_LOG(LOG_VERB, "Notifying all participants\n");
		json_t *destroyed = json_object();
		json_object_set_new(destroyed, "videoroom", json_string("destroyed"));
		json_object_set_new(destroyed, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		janus_mutex_lock(&videoroom->mutex);
		g_hash_table_iter_init(&iter, videoroom->participants);
		while (g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom_publisher *p = value;
			if(p && !g_atomic_int_get(&p->destroyed) && p->session && p->room) {
				g_clear_pointer(&p->room, janus_videoroom_room_dereference);
				/* Notify the user we're going to destroy the room... */
				int ret = gateway->push_event(p->session->handle, &janus_videoroom_plugin, NULL, destroyed, NULL);
				JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
				/* ... and then ask the core to close the PeerConnection */
				gateway->close_pc(p->session->handle);
			}
		}
		json_decref(destroyed);
		janus_mutex_unlock(&videoroom->mutex);
		/* Also notify event handlers */
		if(notify_events && gateway->events_is_enabled()) {
			json_t *info = json_object();
			json_object_set_new(info, "event", json_string("destroyed"));
			json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
			gateway->notify_event(&janus_videoroom_plugin, session ? session->handle : NULL, info);
		}
		janus_mutex_unlock(&rooms_mutex);
		if(save) {
			/* This change is permanent: save to the configuration file too
			 * FIXME: We should check if anything fails... */
			JANUS_LOG(LOG_VERB, "Destroying room %s permanently in config file\n", room_id_str);
			janus_mutex_lock(&config_mutex);
			char cat[BUFSIZ];
			/* The room ID is the category (prefixed by "room-") */
			g_snprintf(cat, BUFSIZ, "room-%s", room_id_str);
			janus_config_remove(config, NULL, cat);
			/* Save modified configuration */
			if(janus_config_save(config, config_folder, JANUS_VIDEOROOM_PACKAGE) < 0)
				save = FALSE;	/* This will notify the user the room destruction is not permanent */
			janus_mutex_unlock(&config_mutex);
		}
		janus_refcount_decrease(&videoroom->ref);
		/* Done */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("destroyed"));
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "permanent", save ? json_true() : json_false());
		goto prepare_response;
	} else if(!strcasecmp(request_text, "list")) {
		/* List all rooms (but private ones) and their details (except for the secret, of course...) */
		JANUS_LOG(LOG_VERB, "Getting the list of VideoRoom rooms\n");
		gboolean lock_room_list = TRUE;
		if(admin_key != NULL) {
			json_t *admin_key_json = json_object_get(root, "admin_key");
			/* Verify admin_key if it was provided */
			if(admin_key_json != NULL && json_is_string(admin_key_json) && strlen(json_string_value(admin_key_json)) > 0) {
				JANUS_CHECK_SECRET(admin_key, root, "admin_key", error_code, error_cause,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
				if(error_code != 0) {
					goto prepare_response;
				} else {
					lock_room_list = FALSE;
				}
			}
		}
		json_t *list = json_array();
		janus_mutex_lock(&rooms_mutex);
		GHashTableIter iter;
		gpointer value;
		g_hash_table_iter_init(&iter, rooms);
		while(g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom *room = value;
			if(!room)
				continue;
			janus_refcount_increase(&room->ref);
			if(room->is_private && lock_room_list) {
				/* Skip private room if no valid admin_key was provided */
				JANUS_LOG(LOG_VERB, "Skipping private room '%s'\n", room->room_name);
				janus_refcount_decrease(&room->ref);
				continue;
			}
			if(!g_atomic_int_get(&room->destroyed)) {
				json_t *rl = json_object();
				json_object_set_new(rl, "room", string_ids ? json_string(room->room_id_str) : json_integer(room->room_id));
				json_object_set_new(rl, "description", json_string(room->room_name));
				json_object_set_new(rl, "pin_required", room->room_pin ? json_true() : json_false());
				json_object_set_new(rl, "is_private", room->is_private ? json_true() : json_false());
				json_object_set_new(rl, "max_publishers", json_integer(room->max_publishers));
				json_object_set_new(rl, "bitrate", json_integer(room->bitrate));
				if(room->bitrate_cap)
					json_object_set_new(rl, "bitrate_cap", json_true());
				json_object_set_new(rl, "fir_freq", json_integer(room->fir_freq));
				json_object_set_new(rl, "require_pvtid", room->require_pvtid ? json_true() : json_false());
				json_object_set_new(rl, "require_e2ee", room->require_e2ee ? json_true() : json_false());
				json_object_set_new(rl, "dummy_publisher", room->dummy_publisher ? json_true() : json_false());
				json_object_set_new(rl, "notify_joining", room->notify_joining ? json_true() : json_false());
				char audio_codecs[100];
				char video_codecs[100];
				janus_videoroom_codecstr(room, audio_codecs, video_codecs, sizeof(audio_codecs), ",");
				json_object_set_new(rl, "audiocodec", json_string(audio_codecs));
				json_object_set_new(rl, "videocodec", json_string(video_codecs));
				if(room->do_opusfec)
					json_object_set_new(rl, "opus_fec", json_true());
				if(room->do_opusdtx)
					json_object_set_new(rl, "opus_dtx", json_true());
				if(room->do_svc)
					json_object_set_new(rl, "video_svc", json_true());
				json_object_set_new(rl, "record", room->record ? json_true() : json_false());
				json_object_set_new(rl, "rec_dir", json_string(room->rec_dir));
				json_object_set_new(rl, "lock_record", room->lock_record ? json_true() : json_false());
				json_object_set_new(rl, "num_participants", json_integer(g_hash_table_size(room->participants)));
				json_object_set_new(rl, "audiolevel_ext", room->audiolevel_ext ? json_true() : json_false());
				json_object_set_new(rl, "audiolevel_event", room->audiolevel_event ? json_true() : json_false());
				if(room->audiolevel_event) {
					json_object_set_new(rl, "audio_active_packets", json_integer(room->audio_active_packets));
					json_object_set_new(rl, "audio_level_average", json_integer(room->audio_level_average));
				}
				json_object_set_new(rl, "videoorient_ext", room->videoorient_ext ? json_true() : json_false());
				json_object_set_new(rl, "playoutdelay_ext", room->playoutdelay_ext ? json_true() : json_false());
				json_object_set_new(rl, "transport_wide_cc_ext", room->transport_wide_cc_ext ? json_true() : json_false());
				json_array_append_new(list, rl);
			}
			janus_refcount_decrease(&room->ref);
		}
		janus_mutex_unlock(&rooms_mutex);
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		json_object_set_new(response, "list", list);
		goto prepare_response;
	} else if(!strcasecmp(request_text, "rtp_forward")) {
		JANUS_VALIDATE_JSON_OBJECT(root, rtp_forward_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, pid_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, pidstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		if(lock_rtpfwd && admin_key != NULL) {
			/* An admin key was specified: make sure it was provided, and that it's valid */
			JANUS_VALIDATE_JSON_OBJECT(root, adminkey_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			if(error_code != 0)
				goto prepare_response;
			JANUS_CHECK_SECRET(admin_key, root, "admin_key", error_code, error_cause,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
			if(error_code != 0)
				goto prepare_response;
		}
		json_t *room = json_object_get(root, "room");
		json_t *pub_id = json_object_get(root, "publisher_id");
		json_t *json_host = json_object_get(root, "host");
		json_t *json_host_family = json_object_get(root, "host_family");
		const char *host_family = json_string_value(json_host_family);
		int family = 0;
		if(host_family) {
			if(!strcasecmp(host_family, "ipv4")) {
				family = AF_INET;
			} else if(!strcasecmp(host_family, "ipv6")) {
				family = AF_INET6;
			} else {
				JANUS_LOG(LOG_ERR, "Unsupported protocol family (%s)\n", host_family);
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
				g_snprintf(error_cause, 512, "Unsupported protocol family (%s)", host_family);
				goto prepare_response;
			}
		}
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		guint64 publisher_id = 0;
		char publisher_id_num[30], *publisher_id_str = NULL;
		if(!string_ids) {
			publisher_id = json_integer_value(pub_id);
			g_snprintf(publisher_id_num, sizeof(publisher_id_num), "%"SCNu64, publisher_id);
			publisher_id_str = publisher_id_num;
		} else {
			publisher_id_str = (char *)json_string_value(pub_id);
		}
		const char *host = json_string_value(json_host), *resolved_host = NULL;
		/* Check if we need to resolve this host address */
		struct addrinfo *res = NULL, *start = NULL;
		janus_network_address addr;
		janus_network_address_string_buffer addr_buf;
		struct addrinfo hints;
		memset(&hints, 0, sizeof(hints));
		if(family != 0)
			hints.ai_family = family;
		if(getaddrinfo(host, NULL, family != 0 ? &hints : NULL, &res) == 0) {
			start = res;
			while(res != NULL) {
				if(janus_network_address_from_sockaddr(res->ai_addr, &addr) == 0 &&
						janus_network_address_to_string_buffer(&addr, &addr_buf) == 0) {
					/* Resolved */
					resolved_host = janus_network_address_string_from_buffer(&addr_buf);
					freeaddrinfo(start);
					start = NULL;
					break;
				}
				res = res->ai_next;
			}
		}
		if(resolved_host == NULL) {
			if(start)
				freeaddrinfo(start);
			JANUS_LOG(LOG_ERR, "Could not resolve address (%s)...\n", host);
			error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
			g_snprintf(error_cause, 512, "Could not resolve address (%s)...", host);
			goto prepare_response;
		}
		host = resolved_host;
		/* Iterate on the provided streams array */
		json_t *streams = json_object_get(root, "streams");
		if(streams == NULL || json_array_size(streams) == 0) {
			/* No streams array, we'll use the legacy approach: make sure the host attribute was set */
			if(host == NULL) {
				error_code = JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT;
				g_snprintf(error_cause, sizeof(error_cause), "Missing mandatory element host (deprecated API)");
				goto prepare_response;
			}
		} else {
			/* Iterate on the streams objects and validate them all */
			size_t i = 0;
			for(i=0; i<json_array_size(streams); i++) {
				json_t *s = json_array_get(streams, i);
				JANUS_VALIDATE_JSON_OBJECT(s, rtp_forward_stream_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0)
					goto prepare_response;
				/* Make sure we have a host attribute, either global or stream-specific */
				json_t *stream_host = json_object_get(s, "host");
				const char *s_host = json_string_value(stream_host);
				if(host == NULL && s_host == NULL) {
					error_code = JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT;
					g_snprintf(error_cause, sizeof(error_cause), "Missing mandatory element host (global or local)");
					goto prepare_response;
				}
				if(s_host != NULL) {
					json_t *stream_host_family = json_object_get(s, "host_family");
					const char *s_host_family = json_string_value(stream_host_family);
					int s_family = family;
					if(s_host_family) {
						if(!strcasecmp(s_host_family, "ipv4")) {
							s_family = AF_INET;
						} else if(!strcasecmp(s_host_family, "ipv6")) {
							s_family = AF_INET6;
						} else {
							JANUS_LOG(LOG_ERR, "Unsupported protocol family (%s)\n", s_host_family);
							error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
							g_snprintf(error_cause, 512, "Unsupported protocol family (%s)", s_host_family);
							goto prepare_response;
						}
					}
					memset(&hints, 0, sizeof(hints));
					if(s_family != 0)
						hints.ai_family = s_family;
					start = NULL;
					res = NULL;
					if(getaddrinfo(s_host, NULL, s_family != 0 ? &hints : NULL, &res) == 0) {
						start = res;
						while(res != NULL) {
							if(janus_network_address_from_sockaddr(res->ai_addr, &addr) == 0 &&
									janus_network_address_to_string_buffer(&addr, &addr_buf) == 0) {
								/* Resolved */
								resolved_host = janus_network_address_string_from_buffer(&addr_buf);
								freeaddrinfo(start);
								start = NULL;
								break;
							}
							res = res->ai_next;
						}
					}
					if(resolved_host == NULL) {
						if(start)
							freeaddrinfo(start);
						JANUS_LOG(LOG_ERR, "Could not resolve address (%s)...\n", s_host);
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Could not resolve address (%s)...", s_host);
						goto prepare_response;
					}
					/* Add the resolved address to the JSON object, so that we can use it later */
					json_object_set_new(s, "host", json_string(resolved_host));
				}
			}
		}
		/* We may need to SRTP-encrypt this stream */
		int srtp_suite = 0;
		const char *srtp_crypto = NULL;
		json_t *s_suite = json_object_get(root, "srtp_suite");
		json_t *s_crypto = json_object_get(root, "srtp_crypto");
		if(s_suite && s_crypto) {
			srtp_suite = json_integer_value(s_suite);
			if(srtp_suite != 32 && srtp_suite != 80) {
				JANUS_LOG(LOG_ERR, "Invalid SRTP suite (%d)\n", srtp_suite);
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
				g_snprintf(error_cause, 512, "Invalid SRTP suite (%d)", srtp_suite);
				goto prepare_response;
			}
			srtp_crypto = json_string_value(s_crypto);
		}
		/* Look for room and publisher */
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		janus_videoroom_publisher *publisher = g_hash_table_lookup(videoroom->participants,
			string_ids ? (gpointer)publisher_id_str : (gpointer)&publisher_id);
		if(publisher == NULL) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			JANUS_LOG(LOG_ERR, "No such publisher (%s)\n", publisher_id_str);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such feed (%s)", publisher_id_str);
			goto prepare_response;
		}
		janus_refcount_increase(&publisher->ref);	/* This is just to handle the request for now */
		janus_mutex_lock(&publisher->rtp_forwarders_mutex);
		if(publisher->udp_sock <= 0) {
			publisher->udp_sock = socket(!ipv6_disabled ? AF_INET6 : AF_INET, SOCK_DGRAM, IPPROTO_UDP);
			int v6only = 0;
			if(publisher->udp_sock <= 0 ||
					(!ipv6_disabled && setsockopt(publisher->udp_sock, IPPROTO_IPV6, IPV6_V6ONLY, &v6only, sizeof(v6only)) != 0)) {
				janus_mutex_unlock(&publisher->rtp_forwarders_mutex);
				janus_refcount_decrease(&publisher->ref);
				janus_mutex_unlock(&videoroom->mutex);
				janus_refcount_decrease(&videoroom->ref);
				JANUS_LOG(LOG_ERR, "Could not open UDP socket for RTP stream for publisher (%s), %d (%s)\n",
					publisher_id_str, errno, g_strerror(errno));
				error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
				g_snprintf(error_cause, 512, "Could not open UDP socket for RTP stream");
				goto prepare_response;
			}
		}
		/* Are we using the new approach, or the old deprecated one? */
		response = json_object();
		janus_videoroom_publisher_stream *ps = NULL;
		json_t *new_forwarders = NULL, *rtp_stream = NULL;
		if(streams != NULL) {
			/* New approach: iterate on all objects, and create the related forwarder(s) */
			new_forwarders = json_array();
			size_t i = 0;
			for(i=0; i<json_array_size(streams); i++) {
				json_t *s = json_array_get(streams, i);
				json_t *stream_mid = json_object_get(s, "mid");
				const char *mid = json_string_value(stream_mid);
				janus_mutex_lock(&publisher->streams_mutex);
				ps = g_hash_table_lookup(publisher->streams_bymid, mid);
				janus_mutex_unlock(&publisher->streams_mutex);
				if(ps == NULL) {
					/* FIXME Should we return an error instead? */
					JANUS_LOG(LOG_WARN, "No such stream with mid '%s', skipping forwarder...\n", mid);
					continue;
				}
				janus_videoroom_rtp_forwarder *f = NULL;
				json_t *stream_host = json_object_get(s, "host");
				host = json_string_value(stream_host) ? json_string_value(stream_host) : json_string_value(json_host);
				json_t *stream_port = json_object_get(s, "port");
				uint16_t port = json_integer_value(stream_port);
				if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA) {
					/* We have all we need */
					f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
						host, port, 0, 0, 0, FALSE, 0, NULL, 0, FALSE, TRUE);
					if(f) {
						json_t *rtpf = janus_videoroom_rtp_forwarder_summary(f);
						json_array_append_new(new_forwarders, rtpf);
						/* Also notify event handlers */
						if(notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room",
								string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("data"));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(port));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
					continue;
				}
				/* If we got here, it's RTP media, check the other properties too */
				json_t *stream_pt = json_object_get(s, "pt");
				json_t *stream_ssrc = json_object_get(s, "ssrc");
				json_t *stream_rtcp_port = json_object_get(s, "rtcp_port");
				if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
					f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
						host, port, stream_rtcp_port ? json_integer_value(stream_rtcp_port) : -1,
						json_integer_value(stream_pt), json_integer_value(stream_ssrc),
						FALSE, srtp_suite, srtp_crypto, 0, FALSE, FALSE);
					if(f) {
						json_t *rtpf = janus_videoroom_rtp_forwarder_summary(f);
						json_array_append_new(new_forwarders, rtpf);
						/* Also notify event handlers */
						if(notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room",
								string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("audio"));
							json_object_set_new(info, "codec", json_string(janus_audiocodec_name(ps->acodec)));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(port));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
				} else {
					json_t *stream_simulcast = json_object_get(s, "simulcast");
					f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
						host, port, stream_rtcp_port ? json_integer_value(stream_rtcp_port) : -1,
						json_integer_value(stream_pt), json_integer_value(stream_ssrc),
						json_is_true(stream_simulcast), srtp_suite, srtp_crypto, 0, TRUE, FALSE);
					if(f) {
						json_t *rtpf = janus_videoroom_rtp_forwarder_summary(f);
						json_array_append_new(new_forwarders, rtpf);
						/* Also notify event handlers */
						if(notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room",
								string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("video"));
							json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(port));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
					if(!json_is_true(stream_simulcast)) {
						/* Check if there's simulcast substreams we need to relay */
						stream_port = json_object_get(s, "port_2");
						port = json_integer_value(stream_port);
						stream_pt = json_object_get(s, "pt_2");
						stream_ssrc = json_object_get(s, "ssrc_2");
						if(json_integer_value(stream_port) > 0) {
							f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
								host, port, 0, json_integer_value(stream_pt), json_integer_value(stream_ssrc),
								FALSE, srtp_suite, srtp_crypto, 1, TRUE, FALSE);
							if(f) {
								json_t *rtpf = janus_videoroom_rtp_forwarder_summary(f);
								json_array_append_new(new_forwarders, rtpf);
								/* Also notify event handlers */
								if(notify_events && gateway->events_is_enabled()) {
									json_t *info = janus_videoroom_rtp_forwarder_summary(f);
									json_object_set_new(info, "event", json_string("rtp_forward"));
									json_object_set_new(info, "room",
										string_ids ? json_string(room_id_str) : json_integer(room_id));
									json_object_set_new(info, "publisher_id",
										string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
									json_object_set_new(info, "media", json_string("video"));
									json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
									json_object_set_new(info, "video_substream", json_integer(1));
									json_object_set_new(info, "stream_id", json_integer(f->stream_id));
									json_object_set_new(info, "host", json_string(host));
									json_object_set_new(info, "port", json_integer(port));
									gateway->notify_event(&janus_videoroom_plugin, NULL, info);
								}
							}
						}
						stream_port = json_object_get(s, "port_3");
						port = json_integer_value(stream_port);
						stream_pt = json_object_get(s, "pt_3");
						stream_ssrc = json_object_get(s, "ssrc_3");
						if(json_integer_value(stream_port) > 0) {
							f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
								host, port, 0, json_integer_value(stream_pt), json_integer_value(stream_ssrc),
								FALSE, srtp_suite, srtp_crypto, 2, TRUE, FALSE);
							if(f) {
								json_t *rtpf = janus_videoroom_rtp_forwarder_summary(f);
								json_array_append_new(new_forwarders, rtpf);
								/* Also notify event handlers */
								if(notify_events && gateway->events_is_enabled()) {
									json_t *info = janus_videoroom_rtp_forwarder_summary(f);
									json_object_set_new(info, "event", json_string("rtp_forward"));
									json_object_set_new(info, "room",
										string_ids ? json_string(room_id_str) : json_integer(room_id));
									json_object_set_new(info, "publisher_id",
										string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
									json_object_set_new(info, "media", json_string("video"));
									json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
									json_object_set_new(info, "video_substream", json_integer(2));
									json_object_set_new(info, "stream_id", json_integer(f->stream_id));
									json_object_set_new(info, "host", json_string(host));
									json_object_set_new(info, "port", json_integer(port));
									gateway->notify_event(&janus_videoroom_plugin, NULL, info);
								}
							}
						}
					}
				}
			}
		} else {
			/* Old deprecated approach: return the legacy info as well */
			JANUS_LOG(LOG_WARN, "Deprecated 'rtp_forward' API: please start looking into the new one for the future\n");
			rtp_stream = json_object();
			int video_port[3] = {-1, -1, -1}, video_rtcp_port = -1, video_pt[3] = {0, 0, 0};
			uint32_t video_ssrc[3] = {0, 0, 0};
			int audio_port = -1, audio_rtcp_port = -1, audio_pt = 0;
			uint32_t audio_ssrc = 0;
			int data_port = -1;
			/* There may be multiple target video ports (e.g., publisher simulcasting) */
			json_t *vid_port = json_object_get(root, "video_port");
			if(vid_port) {
				video_port[0] = json_integer_value(vid_port);
				json_t *pt = json_object_get(root, "video_pt");
				if(pt)
					video_pt[0] = json_integer_value(pt);
				json_t *ssrc = json_object_get(root, "video_ssrc");
				if(ssrc)
					video_ssrc[0] = json_integer_value(ssrc);
			}
			vid_port = json_object_get(root, "video_port_2");
			if(vid_port) {
				video_port[1] = json_integer_value(vid_port);
				json_t *pt = json_object_get(root, "video_pt_2");
				if(pt)
					video_pt[1] = json_integer_value(pt);
				json_t *ssrc = json_object_get(root, "video_ssrc_2");
				if(ssrc)
					video_ssrc[1] = json_integer_value(ssrc);
			}
			vid_port = json_object_get(root, "video_port_3");
			if(vid_port) {
				video_port[2] = json_integer_value(vid_port);
				json_t *pt = json_object_get(root, "video_pt_3");
				if(pt)
					video_pt[2] = json_integer_value(pt);
				json_t *ssrc = json_object_get(root, "video_ssrc_3");
				if(ssrc)
					video_ssrc[2] = json_integer_value(ssrc);
			}
			json_t *vid_rtcp_port = json_object_get(root, "video_rtcp_port");
			if(vid_rtcp_port)
				video_rtcp_port = json_integer_value(vid_rtcp_port);
			/* Audio target */
			json_t *au_port = json_object_get(root, "audio_port");
			if(au_port) {
				audio_port = json_integer_value(au_port);
				json_t *pt = json_object_get(root, "audio_pt");
				if(pt)
					audio_pt = json_integer_value(pt);
				json_t *ssrc = json_object_get(root, "audio_ssrc");
				if(ssrc)
					audio_ssrc = json_integer_value(ssrc);
			}
			json_t *au_rtcp_port = json_object_get(root, "audio_rtcp_port");
			if(au_rtcp_port)
				audio_rtcp_port = json_integer_value(au_rtcp_port);
			/* Data target */
			json_t *d_port = json_object_get(root, "data_port");
			if(d_port) {
				data_port = json_integer_value(d_port);
			}
			/* Do we need to forward multiple simulcast streams to a single endpoint? */
			gboolean simulcast = FALSE;
			if(json_object_get(root, "simulcast") != NULL)
				simulcast = json_is_true(json_object_get(root, "simulcast"));
			if(simulcast) {
				/* We do, disable the other video ports if they were requested */
				video_port[1] = -1;
				video_port[2] = -1;
			}
			/* Create all the forwarders we need */
			janus_videoroom_rtp_forwarder *f = NULL;
			guint32 audio_handle = 0;
			guint32 video_handle[3] = {0, 0, 0};
			guint32 data_handle = 0;
			janus_mutex_lock(&publisher->streams_mutex);
			if(audio_port > 0) {
				/* FIXME Find the audio stream */
				GList *temp = publisher->streams;
				while(temp) {
					ps = (janus_videoroom_publisher_stream *)temp->data;
					if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
						/* FIXME Found */
						break;
					}
					ps = NULL;
					temp = temp->next;
				}
				if(ps == NULL) {
					JANUS_LOG(LOG_WARN, "Couldn't find any audio stream to forward, skipping...\n");
				} else {
					f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
						host, audio_port, audio_rtcp_port, audio_pt, audio_ssrc,
						FALSE, srtp_suite, srtp_crypto, 0, FALSE, FALSE);
					audio_handle = f ? f->stream_id : 0;
					/* Also notify event handlers */
					if(f != NULL && notify_events && gateway->events_is_enabled()) {
						json_t *info = janus_videoroom_rtp_forwarder_summary(f);
						json_object_set_new(info, "event", json_string("rtp_forward"));
						json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
						json_object_set_new(info, "publisher_id",
							string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
						json_object_set_new(info, "media", json_string("audio"));
						json_object_set_new(info, "stream_id", json_integer(f->stream_id));
						json_object_set_new(info, "host", json_string(host));
						json_object_set_new(info, "port", json_integer(audio_port));
						gateway->notify_event(&janus_videoroom_plugin, NULL, info);
					}
				}
			}
			if(video_port[0] > 0 || video_port[1] > 0 || video_port[2] > 0) {
				/* FIXME Find the video stream */
				GList *temp = publisher->streams;
				while(temp) {
					ps = (janus_videoroom_publisher_stream *)temp->data;
					if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
						/* FIXME Found */
						break;
					}
					ps = NULL;
					temp = temp->next;
				}
				if(ps == NULL) {
					JANUS_LOG(LOG_WARN, "Couldn't find any video stream to forward, skipping...\n");
				} else {
					if(video_port[0] > 0) {
						f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
							host, video_port[0], video_rtcp_port, video_pt[0], video_ssrc[0],
							simulcast, srtp_suite, srtp_crypto, 0, TRUE, FALSE);
						video_handle[0] = f ? f->stream_id : 0;
						/* Also notify event handlers */
						if(f != NULL && notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("video"));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(video_port[0]));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
					if(video_port[1] > 0) {
						f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
							host, video_port[1], 0, video_pt[1], video_ssrc[1],
							FALSE, srtp_suite, srtp_crypto, 1, TRUE, FALSE);
						video_handle[1] = f ? f->stream_id : 0;
						/* Also notify event handlers */
						if(f != NULL && notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("video"));
							json_object_set_new(info, "video_substream", json_integer(1));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(video_port[1]));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
					if(video_port[2] > 0) {
						f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
							host, video_port[2], 0, video_pt[2], video_ssrc[2],
							FALSE, srtp_suite, srtp_crypto, 2, TRUE, FALSE);
						video_handle[2] = f ? f->stream_id : 0;
						/* Also notify event handlers */
						if(f != NULL && notify_events && gateway->events_is_enabled()) {
							json_t *info = janus_videoroom_rtp_forwarder_summary(f);
							json_object_set_new(info, "event", json_string("rtp_forward"));
							json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
							json_object_set_new(info, "publisher_id",
								string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
							json_object_set_new(info, "media", json_string("video"));
							json_object_set_new(info, "video_substream", json_integer(2));
							json_object_set_new(info, "stream_id", json_integer(f->stream_id));
							json_object_set_new(info, "host", json_string(host));
							json_object_set_new(info, "port", json_integer(video_port[2]));
							gateway->notify_event(&janus_videoroom_plugin, NULL, info);
						}
					}
					janus_videoroom_reqpli(ps, "New RTP forward publisher");
				}
			}
			if(data_port > 0) {
				/* FIXME Find the data stream */
				GList *temp = publisher->streams;
				while(temp) {
					ps = (janus_videoroom_publisher_stream *)temp->data;
					if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_DATA) {
						/* FIXME Found */
						break;
					}
					ps = NULL;
					temp = temp->next;
				}
				if(ps == NULL) {
					JANUS_LOG(LOG_WARN, "Couldn't find any data stream to forward, skipping...\n");
				} else {
					f = janus_videoroom_rtp_forwarder_add_helper(publisher, ps,
						host, data_port, 0, 0, 0, FALSE, 0, NULL, 0, FALSE, TRUE);
					data_handle = f ? f->stream_id : 0;
					/* Also notify event handlers */
					if(f != NULL && notify_events && gateway->events_is_enabled()) {
						json_t *info = janus_videoroom_rtp_forwarder_summary(f);
						json_object_set_new(info, "event", json_string("rtp_forward"));
						json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
						json_object_set_new(info, "publisher_id",
							string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
						json_object_set_new(info, "media", json_string("data"));
						json_object_set_new(info, "stream_id", json_integer(f->stream_id));
						json_object_set_new(info, "host", json_string(host));
						json_object_set_new(info, "port", json_integer(data_port));
						gateway->notify_event(&janus_videoroom_plugin, NULL, info);
					}
				}
			}
			janus_mutex_unlock(&publisher->streams_mutex);
			if(audio_handle > 0) {
				json_object_set_new(rtp_stream, "audio_stream_id", json_integer(audio_handle));
				json_object_set_new(rtp_stream, "audio", json_integer(audio_port));
			}
			if(video_handle[0] > 0 || video_handle[1] > 0 || video_handle[2] > 0) {
				/* Done */
				if(video_handle[0] > 0) {
					json_object_set_new(rtp_stream, "video_stream_id", json_integer(video_handle[0]));
					json_object_set_new(rtp_stream, "video", json_integer(video_port[0]));
				}
				if(video_handle[1] > 0) {
					json_object_set_new(rtp_stream, "video_stream_id_2", json_integer(video_handle[1]));
					json_object_set_new(rtp_stream, "video_2", json_integer(video_port[1]));
				}
				if(video_handle[2] > 0) {
					json_object_set_new(rtp_stream, "video_stream_id_3", json_integer(video_handle[2]));
					json_object_set_new(rtp_stream, "video_3", json_integer(video_port[2]));
				}
			}
			if(data_handle > 0) {
				json_object_set_new(rtp_stream, "data_stream_id", json_integer(data_handle));
				json_object_set_new(rtp_stream, "data", json_integer(data_port));
			}
			json_object_set_new(rtp_stream, "warning", json_string("deprecated_api"));
		}
		janus_mutex_unlock(&publisher->rtp_forwarders_mutex);
		janus_mutex_unlock(&videoroom->mutex);
		/* These two unrefs are related to the message handling */
		janus_refcount_decrease(&publisher->ref);
		janus_refcount_decrease(&videoroom->ref);
		json_object_set_new(rtp_stream, "host", json_string(host));
		json_object_set_new(response, "publisher_id", string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
		if(new_forwarders != NULL)
			json_object_set_new(response, "forwarders", new_forwarders);
		if(rtp_stream != NULL)
			json_object_set_new(response, "rtp_stream", rtp_stream);
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "videoroom", json_string("rtp_forward"));
		goto prepare_response;
	} else if(!strcasecmp(request_text, "stop_rtp_forward")) {
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, pid_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, pidstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, stop_rtp_forward_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		if(lock_rtpfwd && admin_key != NULL) {
			/* An admin key was specified: make sure it was provided, and that it's valid */
			JANUS_VALIDATE_JSON_OBJECT(root, adminkey_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			if(error_code != 0)
				goto prepare_response;
			JANUS_CHECK_SECRET(admin_key, root, "admin_key", error_code, error_cause,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
			if(error_code != 0)
				goto prepare_response;
		}
		json_t *room = json_object_get(root, "room");
		json_t *pub_id = json_object_get(root, "publisher_id");
		json_t *id = json_object_get(root, "stream_id");

		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		guint64 publisher_id = 0;
		char publisher_id_num[30], *publisher_id_str = NULL;
		if(!string_ids) {
			publisher_id = json_integer_value(pub_id);
			g_snprintf(publisher_id_num, sizeof(publisher_id_num), "%"SCNu64, publisher_id);
			publisher_id_str = publisher_id_num;
		} else {
			publisher_id_str = (char *)json_string_value(pub_id);
		}
		guint32 stream_id = json_integer_value(id);
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		janus_videoroom_publisher *publisher = g_hash_table_lookup(videoroom->participants,
			string_ids ? (gpointer)publisher_id_str : (gpointer)&publisher_id);
		if(publisher == NULL) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			JANUS_LOG(LOG_ERR, "No such publisher (%s)\n", publisher_id_str);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such feed (%s)", publisher_id_str);
			goto prepare_response;
		}
		janus_refcount_increase(&publisher->ref);	/* Just to handle the message now */
		janus_mutex_lock(&publisher->rtp_forwarders_mutex);
		/* Find the forwarder by iterating on all the streams */
		gboolean found = FALSE;
		GList *temp = publisher->streams;
		while(temp) {
			janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
			janus_mutex_lock(&ps->rtp_forwarders_mutex);
			if(g_hash_table_remove(ps->rtp_forwarders, GUINT_TO_POINTER(stream_id))) {
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
				/* Found, remove from global index too */
				g_hash_table_remove(publisher->rtp_forwarders, GUINT_TO_POINTER(stream_id));
				found = TRUE;
				break;
			}
			janus_mutex_unlock(&ps->rtp_forwarders_mutex);
			temp = temp->next;
		}
		janus_mutex_unlock(&publisher->rtp_forwarders_mutex);
		janus_refcount_decrease(&publisher->ref);
		janus_mutex_unlock(&videoroom->mutex);
		janus_refcount_decrease(&videoroom->ref);
		if(!found) {
			JANUS_LOG(LOG_ERR, "No such stream (%"SCNu32")\n", stream_id);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such stream (%"SCNu32")", stream_id);
			goto prepare_response;
		}
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("stop_rtp_forward"));
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "publisher_id", string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
		json_object_set_new(response, "stream_id", json_integer(stream_id));
		/* Also notify event handlers */
		if(notify_events && gateway->events_is_enabled()) {
			json_t *info = json_object();
			json_object_set_new(info, "event", json_string("stop_rtp_forward"));
			json_object_set_new(info, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
			json_object_set_new(info, "publisher_id", string_ids ? json_string(publisher_id_str) : json_integer(publisher_id));
			json_object_set_new(info, "stream_id", json_integer(stream_id));
			gateway->notify_event(&janus_videoroom_plugin, NULL, info);
		}
		goto prepare_response;
	} else if(!strcasecmp(request_text, "exists")) {
		/* Check whether a given room exists or not, returns true/false */
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		gboolean room_exists = g_hash_table_contains(rooms, string_ids ? (gpointer)room_id_str : (gpointer)&room_id);
		janus_mutex_unlock(&rooms_mutex);
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "exists", room_exists ? json_true() : json_false());
		goto prepare_response;
	} else if(!strcasecmp(request_text, "allowed")) {
		JANUS_LOG(LOG_VERB, "Attempt to edit the list of allowed participants in an existing VideoRoom room\n");
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, allowed_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		json_t *action = json_object_get(root, "action");
		json_t *room = json_object_get(root, "room");
		json_t *allowed = json_object_get(root, "allowed");
		const char *action_text = json_string_value(action);
		if(strcasecmp(action_text, "enable") && strcasecmp(action_text, "disable") &&
				strcasecmp(action_text, "add") && strcasecmp(action_text, "remove")) {
			JANUS_LOG(LOG_ERR, "Unsupported action '%s' (allowed)\n", action_text);
			error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
			g_snprintf(error_cause, 512, "Unsupported action '%s' (allowed)", action_text);
			goto prepare_response;
		}
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		/* A secret may be required for this action */
		JANUS_CHECK_SECRET(videoroom->room_secret, root, "secret", error_code, error_cause,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
		if(error_code != 0) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			goto prepare_response;
		}
		if(!strcasecmp(action_text, "enable")) {
			JANUS_LOG(LOG_VERB, "Enabling the check on allowed authorization tokens for room %s\n", room_id_str);
			videoroom->check_allowed = TRUE;
		} else if(!strcasecmp(action_text, "disable")) {
			JANUS_LOG(LOG_VERB, "Disabling the check on allowed authorization tokens for room %s (free entry)\n", room_id_str);
			videoroom->check_allowed = FALSE;
		} else {
			gboolean add = !strcasecmp(action_text, "add");
			if(allowed) {
				/* Make sure the "allowed" array only contains strings */
				gboolean ok = TRUE;
				if(json_array_size(allowed) > 0) {
					size_t i = 0;
					for(i=0; i<json_array_size(allowed); i++) {
						json_t *a = json_array_get(allowed, i);
						if(!a || !json_is_string(a)) {
							ok = FALSE;
							break;
						}
					}
				}
				if(!ok) {
					janus_mutex_unlock(&videoroom->mutex);
					JANUS_LOG(LOG_ERR, "Invalid element in the allowed array (not a string)\n");
					error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
					g_snprintf(error_cause, 512, "Invalid element in the allowed array (not a string)");
					janus_refcount_decrease(&videoroom->ref);
					goto prepare_response;
				}
				size_t i = 0;
				for(i=0; i<json_array_size(allowed); i++) {
					const char *token = json_string_value(json_array_get(allowed, i));
					if(add) {
						if(!g_hash_table_lookup(videoroom->allowed, token))
							g_hash_table_insert(videoroom->allowed, g_strdup(token), GINT_TO_POINTER(TRUE));
					} else {
						g_hash_table_remove(videoroom->allowed, token);
					}
				}
			}
		}
		/* Prepare response */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		json_object_set_new(response, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
		json_t *list = json_array();
		if(strcasecmp(action_text, "disable")) {
			if(g_hash_table_size(videoroom->allowed) > 0) {
				GHashTableIter iter;
				gpointer key;
				g_hash_table_iter_init(&iter, videoroom->allowed);
				while(g_hash_table_iter_next(&iter, &key, NULL)) {
					char *token = key;
					json_array_append_new(list, json_string(token));
				}
			}
			json_object_set_new(response, "allowed", list);
		}
		/* Done */
		janus_mutex_unlock(&videoroom->mutex);
		janus_refcount_decrease(&videoroom->ref);
		JANUS_LOG(LOG_VERB, "VideoRoom room allowed list updated\n");
		goto prepare_response;
	} else if(!strcasecmp(request_text, "kick")) {
		JANUS_LOG(LOG_VERB, "Attempt to kick a participant from an existing VideoRoom room\n");
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, id_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, idstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, kick_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		json_t *id = json_object_get(root, "id");
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		/* A secret may be required for this action */
		JANUS_CHECK_SECRET(videoroom->room_secret, root, "secret", error_code, error_cause,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
		if(error_code != 0) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			goto prepare_response;
		}
		guint64 user_id = 0;
		char user_id_num[30], *user_id_str = NULL;
		if(!string_ids) {
			user_id = json_integer_value(id);
			g_snprintf(user_id_num, sizeof(user_id_num), "%"SCNu64, user_id);
			user_id_str = user_id_num;
		} else {
			user_id_str = (char *)json_string_value(id);
		}
		janus_videoroom_publisher *participant = g_hash_table_lookup(videoroom->participants,
			string_ids ? (gpointer)user_id_str : (gpointer)&user_id);
		if(participant == NULL) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			JANUS_LOG(LOG_ERR, "No such user %s in room %s\n", user_id_str, room_id_str);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such user %s in room %s", user_id_str, room_id_str);
			goto prepare_response;
		}
		janus_refcount_increase(&participant->ref);
		if(participant->kicked) {
			/* Already kicked */
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			janus_refcount_decrease(&participant->ref);
			response = json_object();
			json_object_set_new(response, "videoroom", json_string("success"));
			/* Done */
			goto prepare_response;
		}
		participant->kicked = TRUE;
		g_atomic_int_set(&participant->session->started, 0);
		/* Prepare an event for this */
		json_t *kicked = json_object();
		json_object_set_new(kicked, "videoroom", json_string("event"));
		json_object_set_new(kicked, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
		json_object_set_new(kicked, "leaving", json_string("ok"));
		json_object_set_new(kicked, "reason", json_string("kicked"));
		int ret = gateway->push_event(participant->session->handle, &janus_videoroom_plugin, NULL, kicked, NULL);
		JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
		json_decref(kicked);
		janus_mutex_unlock(&videoroom->mutex);
		/* If this room requires valid private_id values, we can kick subscriptions too */
		if(videoroom->require_pvtid && participant->subscriptions != NULL) {
			/* Iterate on the subscriptions we know this user has */
			janus_mutex_lock(&participant->own_subscriptions_mutex);
			GSList *s = participant->subscriptions;
			while(s) {
				janus_videoroom_subscriber *subscriber = (janus_videoroom_subscriber *)s->data;
				if(subscriber) {
					subscriber->kicked = TRUE;
					/* FIXME We should also close the PeerConnection, but we risk race conditions if we do it here,
					 * so for now we mark the subscriber as kicked and prevent it from getting any media after this */
				}
				s = s->next;
			}
			janus_mutex_unlock(&participant->own_subscriptions_mutex);
		}
		/* This publisher is leaving, tell everybody */
		janus_videoroom_leave_or_unpublish(participant, TRUE, TRUE);
		/* Tell the core to tear down the PeerConnection, hangup_media will do the rest */
		if(participant && !g_atomic_int_get(&participant->destroyed) && participant->session)
			gateway->close_pc(participant->session->handle);
		JANUS_LOG(LOG_INFO, "Kicked user %s from room %s\n", user_id_str, room_id_str);
		/* Prepare response */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		/* Done */
		janus_refcount_decrease(&videoroom->ref);
		janus_refcount_decrease(&participant->ref);
		goto prepare_response;
	} else if(!strcasecmp(request_text, "moderate")) {
		JANUS_LOG(LOG_VERB, "Attempt to moderate a participant as a moderator in an existing VideoRoom room\n");
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, id_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, idstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		JANUS_VALIDATE_JSON_OBJECT(root, moderate_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		json_t *id = json_object_get(root, "id");
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		/* A secret may be required for this action */
		JANUS_CHECK_SECRET(videoroom->room_secret, root, "secret", error_code, error_cause,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
		if(error_code != 0) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			goto prepare_response;
		}
		guint64 user_id = 0;
		char user_id_num[30], *user_id_str = NULL;
		if(!string_ids) {
			user_id = json_integer_value(id);
			g_snprintf(user_id_num, sizeof(user_id_num), "%"SCNu64, user_id);
			user_id_str = user_id_num;
		} else {
			user_id_str = (char *)json_string_value(id);
		}
		janus_videoroom_publisher *participant = g_hash_table_lookup(videoroom->participants,
			string_ids ? (gpointer)user_id_str : (gpointer)&user_id);
		if(participant == NULL) {
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			JANUS_LOG(LOG_ERR, "No such user %s in room %s\n", user_id_str, room_id_str);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such user %s in room %s", user_id_str, room_id_str);
			goto prepare_response;
		}
		janus_refcount_increase(&participant->ref);
		/* Check if there's any media delivery to change */
		const char *mid = json_string_value(json_object_get(root, "mid"));
		gboolean muted = json_is_true(json_object_get(root, "mute"));
		janus_mutex_lock(&participant->streams_mutex);
		/* Subscribe to a specific mid */
		janus_videoroom_publisher_stream *ps = g_hash_table_lookup(participant->streams_bymid, mid);
		if(ps == NULL) {
			janus_mutex_unlock(&participant->streams_mutex);
			janus_refcount_decrease(&participant->ref);
			janus_mutex_unlock(&videoroom->mutex);
			janus_refcount_decrease(&videoroom->ref);
			JANUS_LOG(LOG_ERR, "No such stream %s\n", mid);
			error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
			g_snprintf(error_cause, 512, "No such stream %s", mid);
			goto prepare_response;
		}
		if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO || ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
			if(participant->session && g_atomic_int_get(&participant->session->started) &&
					!muted && ps->active && ps->muted) {
				/* Audio/Video was just resumed, try resetting the RTP headers for viewers */
				janus_mutex_lock(&ps->subscribers_mutex);
				GSList *temp = ps->subscribers;
				while(temp) {
					janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)temp->data;
					if(ss)
						ss->context.seq_reset = TRUE;
					temp = temp->next;
				}
				janus_mutex_unlock(&ps->subscribers_mutex);
			}
		}
		ps->muted = muted;
		janus_mutex_unlock(&participant->streams_mutex);
		/* Prepare an event for this */
		json_t *event = json_object();
		json_object_set_new(event, "videoroom", json_string("event"));
		json_object_set_new(event, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
		json_object_set_new(event, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
		json_object_set_new(event, "mid", json_string(mid));
		json_object_set_new(event, "moderation", muted ? json_string("muted") : json_string("unmuted"));
		/* Notify the speaker this event is related to as well */
		janus_videoroom_notify_participants(participant, event, TRUE);
		json_decref(event);
		/* Also notify event handlers */
		if(notify_events && gateway->events_is_enabled()) {
			json_t *info = json_object();
			json_object_set_new(info, "videoroom", json_string("moderated"));
			json_object_set_new(info, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
			json_object_set_new(info, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
			json_object_set_new(info, "mid", json_string(mid));
			json_object_set_new(info, "moderation", muted ? json_string("muted") : json_string("unmuted"));
			gateway->notify_event(&janus_videoroom_plugin, NULL, info);
		}
		janus_mutex_unlock(&videoroom->mutex);
		/* Prepare response */
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		/* Done */
		janus_refcount_decrease(&videoroom->ref);
		janus_refcount_decrease(&participant->ref);
		goto prepare_response;
	} else if(!strcasecmp(request_text, "listparticipants")) {
		/* List all participants in a room, specifying whether they're publishers or just attendees */
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, FALSE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		/* Return a list of all participants (whether they're publishing or not) */
		json_t *list = json_array();
		GHashTableIter iter;
		gpointer value;
		janus_mutex_lock(&videoroom->mutex);
		g_hash_table_iter_init(&iter, videoroom->participants);
		while (!g_atomic_int_get(&videoroom->destroyed) && g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom_publisher *p = value;
			json_t *pl = json_object();
			json_object_set_new(pl, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
			if(p->display)
				json_object_set_new(pl, "display", json_string(p->display));
			if(p->dummy)
				json_object_set_new(pl, "dummy", json_true());
			json_object_set_new(pl, "publisher", g_atomic_int_get(&p->session->started) ? json_true() : json_false());
			/* To see if the participant is talking, we need to find the audio stream(s) */
			if(g_atomic_int_get(&p->session->started)) {
				gboolean found = FALSE, talking = FALSE;
				GList *temp = p->streams;
				while(temp) {
					janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
					if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO &&
							ps->audio_level_extmap_id > 0) {
						found = TRUE;
						talking |= ps->talking;
					}
					temp = temp->next;
				}
				if(found)
					json_object_set_new(pl, "talking", talking ? json_true() : json_false());
			}
			json_array_append_new(list, pl);
		}
		janus_mutex_unlock(&videoroom->mutex);
		janus_refcount_decrease(&videoroom->ref);
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("participants"));
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "participants", list);
		goto prepare_response;
	} else if(!strcasecmp(request_text, "listforwarders")) {
		/* List all forwarders in a room */
		if(!string_ids) {
			JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		} else {
			JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		}
		if(error_code != 0)
			goto prepare_response;
		json_t *room = json_object_get(root, "room");
		guint64 room_id = 0;
		char room_id_num[30], *room_id_str = NULL;
		if(!string_ids) {
			room_id = json_integer_value(room);
			g_snprintf(room_id_num, sizeof(room_id_num), "%"SCNu64, room_id);
			room_id_str = room_id_num;
		} else {
			room_id_str = (char *)json_string_value(room);
		}
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		/* Return a list of all forwarders */
		json_t *list = json_array();
		GHashTableIter iter;
		gpointer value;
		janus_mutex_lock(&videoroom->mutex);
		g_hash_table_iter_init(&iter, videoroom->participants);
		while (!g_atomic_int_get(&videoroom->destroyed) && g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom_publisher *p = value;
			janus_mutex_lock(&p->rtp_forwarders_mutex);
			if(g_hash_table_size(p->rtp_forwarders) == 0) {
				janus_mutex_unlock(&p->rtp_forwarders_mutex);
				continue;
			}
			json_t *pl = json_object();
			json_object_set_new(pl, "publisher_id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
			if(p->display)
				json_object_set_new(pl, "display", json_string(p->display));
			json_t *flist = json_array();
			/* Iterate on all media streams to see what's being forwarded */
			janus_videoroom_publisher_stream *ps = NULL;
			GList *temp = p->streams;
			while(temp) {
				ps = (janus_videoroom_publisher_stream *)temp->data;
				janus_mutex_lock(&ps->rtp_forwarders_mutex);
				if(g_hash_table_size(ps->rtp_forwarders) == 0) {
					janus_mutex_unlock(&ps->rtp_forwarders_mutex);
					temp = temp->next;
					continue;
				}
				GHashTableIter iter_f;
				gpointer key_f, value_f;
				g_hash_table_iter_init(&iter_f, ps->rtp_forwarders);
				while(g_hash_table_iter_next(&iter_f, &key_f, &value_f)) {
					janus_videoroom_rtp_forwarder *rpv = value_f;
					/* Return a different, media-agnostic, format */
					json_t *fl = janus_videoroom_rtp_forwarder_summary(rpv);
					json_array_append_new(flist, fl);
				}
				janus_mutex_unlock(&ps->rtp_forwarders_mutex);
				temp = temp->next;
			}
			janus_mutex_unlock(&p->rtp_forwarders_mutex);
			json_object_set_new(pl, "forwarders", flist);
			json_array_append_new(list, pl);
		}
		janus_mutex_unlock(&videoroom->mutex);
		janus_refcount_decrease(&videoroom->ref);
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("forwarders"));
		json_object_set_new(response, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
		json_object_set_new(response, "publishers", list);
		goto prepare_response;
	} else if(!strcasecmp(request_text, "enable_recording")) {
		JANUS_VALIDATE_JSON_OBJECT(root, record_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0)
			goto prepare_response;
		json_t *record = json_object_get(root, "record");
		gboolean recording_active = json_is_true(record);
		JANUS_LOG(LOG_VERB, "Enable Recording: %d\n", (recording_active ? 1 : 0));
		/* Lookup room */
		janus_mutex_lock(&rooms_mutex);
		janus_videoroom *videoroom = NULL;
		error_code = janus_videoroom_access_room(root, TRUE, FALSE, &videoroom, error_cause, sizeof(error_cause));
		if(error_code != 0) {
			janus_mutex_unlock(&rooms_mutex);
			goto prepare_response;
		}
		janus_refcount_increase(&videoroom->ref);
		janus_mutex_unlock(&rooms_mutex);
		janus_mutex_lock(&videoroom->mutex);
		/* Set recording status */
		gboolean room_prev_recording_active = recording_active;
		if (room_prev_recording_active != videoroom->record) {
			/* Room recording state has changed */
			videoroom->record = room_prev_recording_active;
			/* Iterate over all participants */
			gpointer value;
			GHashTableIter iter;
			g_hash_table_iter_init(&iter, videoroom->participants);
			while (g_hash_table_iter_next(&iter, NULL, &value)) {
				janus_videoroom_publisher *participant = value;
				if(participant && participant->session) {
					janus_mutex_lock(&participant->rec_mutex);
					gboolean prev_recording_active = participant->recording_active;
					participant->recording_active = recording_active;
					JANUS_LOG(LOG_VERB, "Setting record property: %s (room %s, user %s)\n",
						participant->recording_active ? "true" : "false", participant->room_id_str, participant->user_id_str);
					/* Do we need to do something with the recordings right now? */
					if(participant->recording_active != prev_recording_active) {
						/* Something changed */
						if(!participant->recording_active) {
							/* Not recording (anymore?) */
							janus_videoroom_recorder_close(participant);
						} else if(participant->recording_active && g_atomic_int_get(&participant->session->started)) {
							/* We've started recording, send a PLI and go on */
							GList *temp = participant->streams;
							while(temp) {
								janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
								janus_videoroom_recorder_create(ps);
								if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
									/* Send a PLI */
									janus_videoroom_reqpli(ps, "Recording video");
								}
								temp = temp->next;
							}
						}
					}
					janus_mutex_unlock(&participant->rec_mutex);
				}
			}
		}
		janus_mutex_unlock(&videoroom->mutex);
		janus_refcount_decrease(&videoroom->ref);
		response = json_object();
		json_object_set_new(response, "videoroom", json_string("success"));
		json_object_set_new(response, "record", json_boolean(recording_active));
		goto prepare_response;
	} else {
		/* Not a request we recognize, don't do anything */
		return NULL;
	}

prepare_response:
		{
			if(error_code == 0 && !response) {
				error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
				g_snprintf(error_cause, 512, "Invalid response");
			}
			if(error_code != 0) {
				/* Prepare JSON error event */
				response = json_object();
				json_object_set_new(response, "videoroom", json_string("event"));
				json_object_set_new(response, "error_code", json_integer(error_code));
				json_object_set_new(response, "error", json_string(error_cause));
			}
			return response;
		}

}

struct janus_plugin_result *janus_videoroom_handle_message(janus_plugin_session *handle, char *transaction, json_t *message, json_t *jsep) {
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return janus_plugin_result_new(JANUS_PLUGIN_ERROR, g_atomic_int_get(&stopping) ? "Shutting down" : "Plugin not initialized", NULL);

	/* Pre-parse the message */
	int error_code = 0;
	char error_cause[512];
	json_t *root = message;
	json_t *response = NULL;

	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session) {
		janus_mutex_unlock(&sessions_mutex);
		JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
		error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
		g_snprintf(error_cause, 512, "%s", "No session associated with this handle...");
		goto plugin_response;
	}
	/* Increase the reference counter for this session: we'll decrease it after we handle the message */
	janus_refcount_increase(&session->ref);
	janus_mutex_unlock(&sessions_mutex);
	if(g_atomic_int_get(&session->destroyed)) {
		JANUS_LOG(LOG_ERR, "Session has already been marked as destroyed...\n");
		error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
		g_snprintf(error_cause, 512, "%s", "Session has already been marked as destroyed...");
		goto plugin_response;
	}

	if(message == NULL) {
		JANUS_LOG(LOG_ERR, "No message??\n");
		error_code = JANUS_VIDEOROOM_ERROR_NO_MESSAGE;
		g_snprintf(error_cause, 512, "%s", "No message??");
		goto plugin_response;
	}
	if(!json_is_object(root)) {
		JANUS_LOG(LOG_ERR, "JSON error: not an object\n");
		error_code = JANUS_VIDEOROOM_ERROR_INVALID_JSON;
		g_snprintf(error_cause, 512, "JSON error: not an object");
		goto plugin_response;
	}
	/* Get the request first */
	JANUS_VALIDATE_JSON_OBJECT(root, request_parameters,
		error_code, error_cause, TRUE,
		JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
	if(error_code != 0)
		goto plugin_response;
	json_t *request = json_object_get(root, "request");
	/* Some requests ('create', 'destroy', 'exists', 'list') can be handled synchronously */
	const char *request_text = json_string_value(request);
	/* We have a separate method to process synchronous requests, as those may
	 * arrive from the Admin API as well, and so we handle them the same way */
	response = janus_videoroom_process_synchronous_request(session, root);
	if(response != NULL) {
		/* We got a response, send it back */
		goto plugin_response;
	} else if(!strcasecmp(request_text, "join") || !strcasecmp(request_text, "joinandconfigure") || !strcasecmp(request_text, "update")
			|| !strcasecmp(request_text, "configure") || !strcasecmp(request_text, "publish") || !strcasecmp(request_text, "unpublish")
			|| !strcasecmp(request_text, "start") || !strcasecmp(request_text, "pause") || !strcasecmp(request_text, "switch")
			|| !strcasecmp(request_text, "subscribe") || !strcasecmp(request_text, "unsubscribe") || !strcasecmp(request_text, "leave")) {
		/* These messages are handled asynchronously */

		janus_videoroom_message *msg = g_malloc(sizeof(janus_videoroom_message));
		msg->handle = handle;
		msg->transaction = transaction;
		msg->message = root;
		msg->jsep = jsep;
		g_async_queue_push(messages, msg);

		return janus_plugin_result_new(JANUS_PLUGIN_OK_WAIT, NULL, NULL);
	} else {
		JANUS_LOG(LOG_VERB, "Unknown request '%s'\n", request_text);
		error_code = JANUS_VIDEOROOM_ERROR_INVALID_REQUEST;
		g_snprintf(error_cause, 512, "Unknown request '%s'", request_text);
	}

plugin_response:
		{
			if(error_code == 0 && !response) {
				error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
				g_snprintf(error_cause, 512, "Invalid response");
			}
			if(error_code != 0) {
				/* Prepare JSON error event */
				json_t *event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "error_code", json_integer(error_code));
				json_object_set_new(event, "error", json_string(error_cause));
				response = event;
			}
			if(root != NULL)
				json_decref(root);
			if(jsep != NULL)
				json_decref(jsep);
			g_free(transaction);

			if(session != NULL)
				janus_refcount_decrease(&session->ref);
			return janus_plugin_result_new(JANUS_PLUGIN_OK, NULL, response);
		}

}

json_t *janus_videoroom_handle_admin_message(json_t *message) {
	/* Some requests (e.g., 'create' and 'destroy') can be handled via Admin API */
	int error_code = 0;
	char error_cause[512];
	json_t *response = NULL;

	JANUS_VALIDATE_JSON_OBJECT(message, request_parameters,
		error_code, error_cause, TRUE,
		JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
	if(error_code != 0)
		goto admin_response;
	json_t *request = json_object_get(message, "request");
	const char *request_text = json_string_value(request);
	if((response = janus_videoroom_process_synchronous_request(NULL, message)) != NULL) {
		/* We got a response, send it back */
		goto admin_response;
	} else {
		JANUS_LOG(LOG_VERB, "Unknown request '%s'\n", request_text);
		error_code = JANUS_VIDEOROOM_ERROR_INVALID_REQUEST;
		g_snprintf(error_cause, 512, "Unknown request '%s'", request_text);
	}

admin_response:
		{
			if(!response) {
				/* Prepare JSON error event */
				response = json_object();
				json_object_set_new(response, "videoroom", json_string("event"));
				json_object_set_new(response, "error_code", json_integer(error_code));
				json_object_set_new(response, "error", json_string(error_cause));
			}
			return response;
		}

}

void janus_videoroom_setup_media(janus_plugin_session *handle) {
	JANUS_LOG(LOG_INFO, "[%s-%p] WebRTC media is now available\n", JANUS_VIDEOROOM_PACKAGE, handle);
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session) {
		janus_mutex_unlock(&sessions_mutex);
		JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
		return;
	}
	if(g_atomic_int_get(&session->destroyed)) {
		janus_mutex_unlock(&sessions_mutex);
		return;
	}
	janus_refcount_increase(&session->ref);
	g_atomic_int_set(&session->hangingup, 0);
	janus_mutex_unlock(&sessions_mutex);

	/* Media relaying can start now */
	g_atomic_int_set(&session->started, 1);
	if(session->participant) {
		/* If this is a publisher, notify all subscribers about the fact they can
		 * now subscribe; if this is a subscriber, instead, ask the publisher a FIR */
		if(session->participant_type == janus_videoroom_p_type_publisher) {
			janus_videoroom_publisher *participant = janus_videoroom_session_get_publisher(session);
			/* Notify all other participants that there's a new boy in town */
			janus_videoroom_notify_about_publisher(participant, FALSE);
			/* Check if we need to start recording */
			janus_mutex_lock(&participant->rec_mutex);
			if((participant->room && participant->room->record) || participant->recording_active) {
				GList *temp = participant->streams;
				while(temp) {
					janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
					janus_videoroom_recorder_create(ps);
					temp = temp->next;
				}
				participant->recording_active = TRUE;
			}
			janus_mutex_unlock(&participant->rec_mutex);
			janus_refcount_decrease(&participant->ref);
		} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
			janus_videoroom_subscriber *s = janus_videoroom_session_get_subscriber(session);
			if(s && s->streams) {
				/* Send a PLI for all the video streams we subscribed to */
				GList *temp = s->streams;
				while(temp) {
					janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)temp->data;
					janus_videoroom_publisher_stream *ps = ss->publisher_streams ? ss->publisher_streams->data : NULL;
					if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && ps->publisher && ps->publisher->session) {
						janus_videoroom_reqpli(ps, "New subscriber available");
					}
					temp = temp->next;
				}
				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("subscribed"));
					json_object_set_new(info, "room", json_integer(s->room_id));
					/* TODO Fix the event to event handlers, we don't have a single feed anymore */
					//~ json_object_set_new(info, "feed", json_integer(p->user_id));
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
			}
			if(s)
				janus_refcount_decrease(&s->ref);
		}
	}
	janus_refcount_decrease(&session->ref);
}

void janus_videoroom_incoming_rtp(janus_plugin_session *handle, janus_plugin_rtp *pkt) {
	if(handle == NULL || g_atomic_int_get(&handle->stopped) || g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session || g_atomic_int_get(&session->destroyed) || session->participant_type != janus_videoroom_p_type_publisher)
		return;
	janus_videoroom_publisher *participant = janus_videoroom_session_get_publisher_nodebug(session);
	if(participant == NULL)
		return;
	if(g_atomic_int_get(&participant->destroyed) || participant->kicked || !participant->streams || participant->room == NULL) {
		janus_videoroom_publisher_dereference_nodebug(participant);
		return;
	}
	janus_videoroom *videoroom = participant->room;

	/* Find the stream this packet belongs to */
	janus_mutex_lock(&participant->streams_mutex);
	janus_videoroom_publisher_stream *ps = g_hash_table_lookup(participant->streams_byid, GINT_TO_POINTER(pkt->mindex));
	janus_mutex_unlock(&participant->streams_mutex);
	if(ps == NULL) {
		/* No stream..? */
		janus_videoroom_publisher_dereference_nodebug(participant);
		return;
	}

	gboolean video = pkt->video;
	char *buf = pkt->buffer;
	uint16_t len = pkt->length;
	janus_videoroom_stat_on_packet(participant->stat + video, len, now_ms());
	/* In case this is an audio packet and we're doing talk detection, check the audio level extension */
	if(!video && videoroom->audiolevel_event && ps->active && !ps->muted && ps->audio_level_extmap_id > 0) {
		int level = pkt->extensions.audio_level;
		if(level != -1) {
			ps->audio_dBov_sum += level;
			ps->audio_active_packets++;
			ps->audio_dBov_level = level;
			int audio_active_packets = participant->user_audio_active_packets ? participant->user_audio_active_packets : videoroom->audio_active_packets;
			int audio_level_average = participant->user_audio_level_average ? participant->user_audio_level_average : videoroom->audio_level_average;
			if(ps->audio_active_packets > 0 && ps->audio_active_packets == audio_active_packets) {
				gboolean notify_talk_event = FALSE;
				float audio_dBov_avg = (float)ps->audio_dBov_sum/(float)ps->audio_active_packets;
				if(audio_dBov_avg < audio_level_average) {
					/* Participant talking, should we notify all participants? */
					if(!ps->talking)
						notify_talk_event = TRUE;
					ps->talking = TRUE;
					ps->last_talking_event_report_time = now_ms();
				} else if (now_ms() - ps->last_talking_event_report_time > AUDIO_EVENT_REPORT_INTERVAL_MS) {
					/* Participant not talking anymore, should we notify all participants? */
					if(ps->talking)
						notify_talk_event = TRUE;
					ps->talking = FALSE;
				}
				ps->audio_active_packets = 0;
				ps->audio_dBov_sum = 0;
				/* Only notify in case of state changes */
				if(notify_talk_event) {
					janus_mutex_lock(&videoroom->mutex);
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string(ps->talking ? "talking" : "stopped-talking"));
					json_object_set_new(event, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
					json_object_set_new(event, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
					json_object_set_new(event, "mindex", json_integer(ps->mindex));
					json_object_set_new(event, "mid", json_string(ps->mid));
					json_object_set_new(event, "audio-level-dBov-avg", json_real(audio_dBov_avg));
					/* Notify the speaker this event is related to as well */
					janus_videoroom_notify_participants(participant, event, TRUE);
					json_decref(event);
					janus_mutex_unlock(&videoroom->mutex);
					/* Also notify event handlers */
					if(notify_events && gateway->events_is_enabled()) {
						json_t *info = json_object();
						json_object_set_new(info, "videoroom", json_string(ps->talking ? "talking" : "stopped-talking"));
						json_object_set_new(info, "room", string_ids ? json_string(videoroom->room_id_str) : json_integer(videoroom->room_id));
						json_object_set_new(info, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
						json_object_set_new(info, "mindex", json_integer(ps->mindex));
						json_object_set_new(info, "mid", json_string(ps->mid));
						json_object_set_new(info, "audio-level-dBov-avg", json_real(audio_dBov_avg));
						gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
					}
				}
			}
		}
	}

	if (!video && ps->active) {
		if (g_hash_table_size(videoroom->participants) >= videoroom->min_audio_mix_participants) {
			speech_mix_incoming_rtp(videoroom, &participant->speech_mix_participant, buf, len, participant->display);
			if (g_atomic_int_compare_and_exchange(&videoroom->audio_mix_active, FALSE, TRUE)) {
				JANUS_LOG(LOG_INFO, "Enable audio mix for room %"SCNu64" (%s)...\n", videoroom->room_id, videoroom->room_name);
			}
		} else {
			if (g_atomic_int_compare_and_exchange(&videoroom->audio_mix_active, TRUE, FALSE)) {
				JANUS_LOG(LOG_INFO, "Disable audio mix for room %"SCNu64" (%s)...\n", videoroom->room_id, videoroom->room_name);
			}
		}
	}

	if(ps->active && !ps->muted) {
		janus_rtp_header *rtp = (janus_rtp_header *)buf;
		int sc = video ? 0 : -1;
		/* Check if we're simulcasting, and if so, keep track of the "layer" */
		if(video && ps->simulcast) {
			uint32_t ssrc = ntohl(rtp->ssrc);
			if(ssrc == ps->vssrc[0])
				sc = 0;
			else if(ssrc == ps->vssrc[1])
				sc = 1;
			else if(ssrc == ps->vssrc[2])
				sc = 2;
			else if(ps->rid_extmap_id > 0) {
				/* We may not know the SSRC yet, try the rid RTP extension */
				char sdes_item[16];
				janus_mutex_lock(&ps->rid_mutex);
				if(janus_rtp_header_extension_parse_rid(buf, len, ps->rid_extmap_id, sdes_item, sizeof(sdes_item)) == 0) {
					if(ps->rid[0] != NULL && !strcmp(ps->rid[0], sdes_item)) {
						ps->vssrc[0] = ssrc;
						sc = 0;
					} else if(ps->rid[1] != NULL && !strcmp(ps->rid[1], sdes_item)) {
						ps->vssrc[1] = ssrc;
						sc = 1;
					} else if(ps->rid[2] != NULL && !strcmp(ps->rid[2], sdes_item)) {
						ps->vssrc[2] = ssrc;
						sc = 2;
					}
				}
				janus_mutex_unlock(&ps->rid_mutex);
			}
		}
		/* Forward RTP to the appropriate port for the rtp_forwarders associated with this publisher, if there are any */
		janus_mutex_lock(&ps->rtp_forwarders_mutex);
		if(participant->srtp_contexts && g_hash_table_size(participant->srtp_contexts) > 0) {
			GHashTableIter iter;
			gpointer value;
			g_hash_table_iter_init(&iter, participant->srtp_contexts);
			while(g_hash_table_iter_next(&iter, NULL, &value)) {
				janus_videoroom_srtp_context *srtp_ctx = (janus_videoroom_srtp_context *)value;
				srtp_ctx->slen = 0;
			}
		}
		GHashTableIter iter;
		gpointer value;
		g_hash_table_iter_init(&iter, ps->rtp_forwarders);
		while(participant->udp_sock > 0 && g_hash_table_iter_next(&iter, NULL, &value)) {
			janus_videoroom_rtp_forwarder *rtp_forward = (janus_videoroom_rtp_forwarder *)value;
			if(rtp_forward->is_data || (video && !rtp_forward->is_video) || (!video && rtp_forward->is_video))
				continue;
			/* Backup the RTP header info, as we may rewrite part of it */
			uint32_t seq_number = ntohs(rtp->seq_number);
			uint32_t timestamp = ntohl(rtp->timestamp);
			int pt = rtp->type;
			uint32_t ssrc = ntohl(rtp->ssrc);
			/* First of all, check if we're simulcasting and if we need to forward or ignore this frame */
			if(video && !rtp_forward->simulcast && rtp_forward->substream != sc) {
				continue;
			} else if(video && rtp_forward->simulcast) {
				/* This is video and we're simulcasting, check if we need to forward this frame */
				if(!janus_rtp_simulcasting_context_process_rtp(&rtp_forward->sim_context,
						buf, len, ps->vssrc, ps->rid, ps->vcodec, &rtp_forward->context, &ps->rid_mutex))
					continue;
				janus_rtp_header_update(rtp, &rtp_forward->context, TRUE, 0);
				/* By default we use a fixed SSRC (it may be overwritten later) */
				rtp->ssrc = htonl(participant->user_id & 0xffffffff);
			}
			/* Check if payload type and/or SSRC need to be overwritten for this forwarder */
			if(rtp_forward->payload_type > 0)
				rtp->type = rtp_forward->payload_type;
			if(rtp_forward->ssrc > 0)
				rtp->ssrc = htonl(rtp_forward->ssrc);
			/* Check if this is an RTP or SRTP forwarder */
			if(!rtp_forward->is_srtp) {
				/* Plain RTP */
				struct sockaddr *address = (rtp_forward->serv_addr.sin_family == AF_INET ?
					(struct sockaddr *)&rtp_forward->serv_addr : (struct sockaddr *)&rtp_forward->serv_addr6);
				size_t addrlen = (rtp_forward->serv_addr.sin_family == AF_INET ? sizeof(rtp_forward->serv_addr) : sizeof(rtp_forward->serv_addr6));
				if(sendto(participant->udp_sock, buf, len, 0, address, addrlen) < 0) {
					JANUS_LOG(LOG_HUGE, "Error forwarding RTP %s packet for %s... %s (len=%d)...\n",
						(video ? "video" : "audio"), participant->display, g_strerror(errno), len);
				}
			} else {
				/* SRTP: check if we already encrypted the packet before */
				if(rtp_forward->srtp_ctx->slen == 0) {
					memcpy(&rtp_forward->srtp_ctx->sbuf, buf, len);
					int protected = len;
					int res = srtp_protect(rtp_forward->srtp_ctx->ctx, &rtp_forward->srtp_ctx->sbuf, &protected);
					if(res != srtp_err_status_ok) {
						janus_rtp_header *header = (janus_rtp_header *)&rtp_forward->srtp_ctx->sbuf;
						guint32 timestamp = ntohl(header->timestamp);
						guint16 seq = ntohs(header->seq_number);
						JANUS_LOG(LOG_ERR, "Error encrypting %s packet for %s... %s (len=%d-->%d, ts=%"SCNu32", seq=%"SCNu16")...\n",
							(video ? "Video" : "Audio"), participant->display, janus_srtp_error_str(res), len, protected, timestamp, seq);
					} else {
						rtp_forward->srtp_ctx->slen = protected;
					}
				}
				if(rtp_forward->srtp_ctx->slen > 0) {
					struct sockaddr *address = (rtp_forward->serv_addr.sin_family == AF_INET ?
						(struct sockaddr *)&rtp_forward->serv_addr : (struct sockaddr *)&rtp_forward->serv_addr6);
					size_t addrlen = (rtp_forward->serv_addr.sin_family == AF_INET ? sizeof(rtp_forward->serv_addr) : sizeof(rtp_forward->serv_addr6));
					if(sendto(participant->udp_sock, rtp_forward->srtp_ctx->sbuf, rtp_forward->srtp_ctx->slen, 0, address, addrlen) < 0) {
						JANUS_LOG(LOG_HUGE, "Error forwarding SRTP %s packet for %s... %s (len=%d)...\n",
							(video ? "video" : "audio"), participant->display, g_strerror(errno), rtp_forward->srtp_ctx->slen);
					}
				}
			}
			/* Restore original values of payload type and SSRC before going on */
			rtp->type = pt;
			rtp->ssrc = htonl(ssrc);
			rtp->timestamp = htonl(timestamp);
			rtp->seq_number = htons(seq_number);
		}
		janus_mutex_unlock(&ps->rtp_forwarders_mutex);
		/* Set the payload type of the publisher */
		rtp->type = ps->pt;
		/* Save the frame if we're recording */
		if(!video || !ps->simulcast) {
			janus_recorder_save_frame(ps->rc, buf, len);
		} else {
			/* We're simulcasting, save the best video quality */
			gboolean save = janus_rtp_simulcasting_context_process_rtp(&ps->rec_simctx,
				buf, len, ps->vssrc, ps->rid, ps->vcodec, &ps->rec_ctx, &ps->rid_mutex);
			if(save) {
				uint32_t seq_number = ntohs(rtp->seq_number);
				uint32_t timestamp = ntohl(rtp->timestamp);
				uint32_t ssrc = ntohl(rtp->ssrc);
				janus_rtp_header_update(rtp, &ps->rec_ctx, TRUE, 0);
				/* We use a fixed SSRC for the whole recording */
				rtp->ssrc = ps->vssrc[0];
				janus_recorder_save_frame(ps->rc, buf, len);
				/* Restore the header, as it will be needed by subscribers */
				rtp->ssrc = htonl(ssrc);
				rtp->timestamp = htonl(timestamp);
				rtp->seq_number = htons(seq_number);
			}
		}
		/* Done, relay it */
		janus_videoroom_rtp_relay_packet packet = { 0 };
		packet.source = ps;
		packet.data = rtp;
		packet.length = len;
		packet.extensions = pkt->extensions;
		packet.is_rtp = TRUE;
		packet.is_video = video;
		packet.svc = FALSE;
		if(video && ps->svc) {
			/* We're doing SVC: let's parse this packet to see which layers are there */
			int plen = 0;
			char *payload = janus_rtp_payload(buf, len, &plen);
			if(payload == NULL) {
				janus_videoroom_publisher_dereference_nodebug(participant);
				return;
			}
			gboolean found = FALSE;
			memset(&packet.svc_info, 0, sizeof(packet.svc_info));
			if(janus_vp9_parse_svc(payload, plen, &found, &packet.svc_info) == 0) {
				packet.svc = found;
			}
		}
		if(video && ps->simulcast)
			packet.simulcast = TRUE;
		packet.ssrc[0] = (sc != -1 ? ps->vssrc[0] : 0);
		packet.ssrc[1] = (sc != -1 ? ps->vssrc[1] : 0);
		packet.ssrc[2] = (sc != -1 ? ps->vssrc[2] : 0);
		/* Backup the actual timestamp and sequence number set by the publisher, in case switching is involved */
		packet.timestamp = ntohl(packet.data->timestamp);
		packet.seq_number = ntohs(packet.data->seq_number);
		if(ps->min_delay > -1 && ps->max_delay > -1) {
			packet.extensions.min_delay = ps->min_delay;
			packet.extensions.max_delay = ps->max_delay;
		}
		/* Go: some viewers may decide to drop the packet, but that's up to them */
		janus_mutex_lock_nodebug(&ps->subscribers_mutex);
		g_slist_foreach(ps->subscribers, janus_videoroom_relay_rtp_packet, &packet);
		janus_mutex_unlock_nodebug(&ps->subscribers_mutex);

		/* Check if we need to send any REMB, FIR or PLI back to this publisher */
		if(video && ps->active && !ps->muted) {
			/* Did we send a REMB already, or is it time to send one? */
			gboolean send_remb = FALSE;
			if(participant->remb_latest == 0 && participant->remb_startup > 0) {
				/* Still in the starting phase, send the ramp-up REMB feedback */
				send_remb = TRUE;
			} else if(participant->remb_latest > 0 && janus_get_monotonic_time()-participant->remb_latest >= 5*G_USEC_PER_SEC) {
				/* 5 seconds have passed since the last REMB, send a new one */
				send_remb = TRUE;
			}

			if(send_remb && participant->bitrate) {
				/* We send a few incremental REMB messages at startup */
				uint32_t bitrate = participant->bitrate;
				if(participant->remb_startup > 0) {
					bitrate = bitrate/participant->remb_startup;
					participant->remb_startup--;
				}
				JANUS_LOG(LOG_VERB, "Sending REMB (%s, %"SCNu32")\n", participant->display, bitrate);
				gateway->send_remb(handle, bitrate);
				if(participant->remb_startup == 0)
					participant->remb_latest = janus_get_monotonic_time();
			}
			/* Generate FIR/PLI too, if needed */
			if(video && ps->active && !ps->muted && (videoroom->fir_freq > 0)) {
				/* We generate RTCP every tot seconds/frames */
				gint64 now = janus_get_monotonic_time();
				/* First check if this is a keyframe, though: if so, we reset the timer */
				int plen = 0;
				char *payload = janus_rtp_payload(buf, len, &plen);
				if(payload == NULL) {
					janus_videoroom_publisher_dereference_nodebug(participant);
					return;
				}
				if(ps->vcodec == JANUS_VIDEOCODEC_VP8) {
					if(janus_vp8_is_keyframe(payload, plen))
						ps->fir_latest = now;
				} else if(ps->vcodec == JANUS_VIDEOCODEC_VP9) {
					if(janus_vp9_is_keyframe(payload, plen))
						ps->fir_latest = now;
				} else if(ps->vcodec == JANUS_VIDEOCODEC_H264) {
					if(janus_h264_is_keyframe(payload, plen))
						ps->fir_latest = now;
				} else if(ps->vcodec == JANUS_VIDEOCODEC_AV1) {
					if(janus_av1_is_keyframe(payload, plen))
						ps->fir_latest = now;
				} else if(ps->vcodec == JANUS_VIDEOCODEC_H265) {
					if(janus_h265_is_keyframe(payload, plen))
						ps->fir_latest = now;
				}
				if((now-ps->fir_latest) >= ((gint64)videoroom->fir_freq*G_USEC_PER_SEC)) {
					/* FIXME We send a FIR every tot seconds */
					janus_videoroom_reqpli(ps, "Regular keyframe request");
				}
			}
		}
	}
	janus_videoroom_publisher_dereference_nodebug(participant);
}

void janus_videoroom_incoming_rtcp(janus_plugin_session *handle, janus_plugin_rtcp *packet) {
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session) {
		JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
		return;
	}
	if(g_atomic_int_get(&session->destroyed))
		return;
	char *buf = packet->buffer;
	uint16_t len = packet->length;
	if(session->participant_type == janus_videoroom_p_type_subscriber) {
		/* A subscriber sent some RTCP, check what it is and if we need to forward it to the publisher */
		janus_videoroom_subscriber *s = janus_videoroom_session_get_subscriber_nodebug(session);
		if(s == NULL)
			return;
		if(g_atomic_int_get(&s->destroyed)) {
			janus_refcount_decrease_nodebug(&s->ref);
			return;
		}
		/* Find the stream this packet belongs to */
		janus_mutex_lock(&s->streams_mutex);
		janus_videoroom_subscriber_stream *ss = g_hash_table_lookup(s->streams_byid, GINT_TO_POINTER(packet->mindex));
		if(ss == NULL || ss->publisher_streams == NULL) {
			/* No stream..? */
			janus_mutex_unlock(&s->streams_mutex);
			janus_refcount_decrease_nodebug(&s->ref);
			return;
		}
		janus_videoroom_publisher_stream *ps = ss->publisher_streams ? ss->publisher_streams->data : NULL;
		if(ps == NULL || ps->type != JANUS_VIDEOROOM_MEDIA_VIDEO) {
			janus_mutex_unlock(&s->streams_mutex);
			janus_refcount_decrease_nodebug(&s->ref);
			return;		/* The only feedback we handle is video related anyway... */
		}
		janus_refcount_increase_nodebug(&ps->ref);
		janus_mutex_unlock(&s->streams_mutex);
		if(janus_rtcp_has_fir(buf, len) || janus_rtcp_has_pli(buf, len)) {
			/* We got a FIR or PLI, forward a PLI to the publisher */
			janus_videoroom_publisher *p = ps->publisher;
			if(p && p->session)
				janus_videoroom_reqpli(ps, "PLI from subscriber");
		}
		uint32_t bitrate = janus_rtcp_get_remb(buf, len);
		if(bitrate > 0) {
			/* FIXME We got a REMB from this subscriber, should we do something about it? */
		}
		janus_refcount_decrease_nodebug(&ps->ref);
		janus_refcount_decrease_nodebug(&s->ref);
	}
}

void janus_videoroom_incoming_data(janus_plugin_session *handle, janus_plugin_data *packet) {
	if(handle == NULL || g_atomic_int_get(&handle->stopped) || g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	if(packet->buffer == NULL || packet->length == 0)
		return;
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session || g_atomic_int_get(&session->destroyed) || session->participant_type != janus_videoroom_p_type_publisher)
		return;
	janus_videoroom_publisher *participant = janus_videoroom_session_get_publisher_nodebug(session);
	if(participant == NULL)
		return;
	if(g_atomic_int_get(&participant->destroyed) || participant->data_mindex < 0 || !participant->streams || participant->kicked) {
		janus_videoroom_publisher_dereference_nodebug(participant);
		return;
	}
	char *buf = packet->buffer;
	uint16_t len = packet->length;

	/* Find the stream this packet belongs to */
	janus_mutex_lock(&participant->streams_mutex);
	janus_videoroom_publisher_stream *ps = g_hash_table_lookup(participant->streams_byid, GINT_TO_POINTER(participant->data_mindex));
	janus_mutex_unlock(&participant->streams_mutex);
	if(ps == NULL || !ps->active || ps->muted) {
		/* No or inactive stream..? */
		janus_videoroom_publisher_dereference_nodebug(participant);
		return;
	}

	/* Any forwarder involved? */
	janus_mutex_lock(&ps->rtp_forwarders_mutex);
	/* Forward RTP to the appropriate port for the rtp_forwarders associated with this publisher, if there are any */
	GHashTableIter iter;
	gpointer value;
	g_hash_table_iter_init(&iter, ps->rtp_forwarders);
	while(participant->udp_sock > 0 && g_hash_table_iter_next(&iter, NULL, &value)) {
		janus_videoroom_rtp_forwarder* rtp_forward = (janus_videoroom_rtp_forwarder*)value;
		if(rtp_forward->is_data) {
			struct sockaddr *address = (rtp_forward->serv_addr.sin_family == AF_INET ?
				(struct sockaddr *)&rtp_forward->serv_addr : (struct sockaddr *)&rtp_forward->serv_addr6);
			size_t addrlen = (rtp_forward->serv_addr.sin_family == AF_INET ? sizeof(rtp_forward->serv_addr) : sizeof(rtp_forward->serv_addr6));
			if(sendto(participant->udp_sock, buf, len, 0, address, addrlen) < 0) {
				JANUS_LOG(LOG_HUGE, "Error forwarding data packet for %s... %s (len=%d)...\n",
					participant->display, g_strerror(errno), len);
			}
		}
	}
	janus_mutex_unlock(&ps->rtp_forwarders_mutex);
	JANUS_LOG(LOG_VERB, "Got a %s DataChannel message (%d bytes) to forward\n",
		packet->binary ? "binary" : "text", len);
	/* Save the message if we're recording */
	janus_recorder_save_frame(ps->rc, buf, len);
	/* Relay to all subscribers */
	janus_videoroom_rtp_relay_packet pkt = { 0 };
	pkt.source = ps;
	pkt.data = (struct rtp_header *)buf;
	pkt.length = len;
	pkt.is_rtp = FALSE;
	pkt.textdata = !packet->binary;
	janus_mutex_lock_nodebug(&ps->subscribers_mutex);
	g_slist_foreach(ps->subscribers, janus_videoroom_relay_data_packet, &pkt);
	janus_mutex_unlock_nodebug(&ps->subscribers_mutex);
	janus_videoroom_publisher_dereference_nodebug(participant);
}

void janus_videoroom_data_ready(janus_plugin_session *handle) {
	if(handle == NULL || g_atomic_int_get(&handle->stopped) ||
			g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized) || !gateway)
		return;
	/* Data channels are writable */
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session || g_atomic_int_get(&session->destroyed) || g_atomic_int_get(&session->hangingup))
		return;
	if(g_atomic_int_compare_and_exchange(&session->dataready, 0, 1)) {
		JANUS_LOG(LOG_INFO, "[%s-%p] Data channel available\n", JANUS_VIDEOROOM_PACKAGE, handle);
	}
}

void janus_videoroom_slow_link(janus_plugin_session *handle, int mindex, gboolean video, gboolean uplink) {
	/* The core is informing us that our peer got too many NACKs, are we pushing media too hard? */
	if(handle == NULL || g_atomic_int_get(&handle->stopped) || g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session || g_atomic_int_get(&session->destroyed) || !session->participant) {
		janus_mutex_unlock(&sessions_mutex);
		return;
	}
	janus_refcount_increase(&session->ref);
	janus_mutex_unlock(&sessions_mutex);
	/* Check if it's an uplink (publisher) or downlink (viewer) issue */
	if(session->participant_type == janus_videoroom_p_type_publisher) {
		if(!uplink) {
			janus_videoroom_publisher *publisher = janus_videoroom_session_get_publisher(session);
			if(publisher == NULL) {
				janus_refcount_decrease(&session->ref);
				return;
			}
			if(g_atomic_int_get(&publisher->destroyed)) {
				janus_refcount_decrease(&publisher->ref);
				janus_refcount_decrease(&session->ref);
				return;
			}
			/* Send an event on the handle to notify the application: it's
			 * up to the application to then choose a policy and enforce it */
			json_t *event = json_object();
			json_object_set_new(event, "videoroom", json_string("slow_link"));
			/* Also add info on what the current bitrate cap is */
			uint32_t bitrate = publisher->bitrate;
			json_object_set_new(event, "current-bitrate", json_integer(bitrate));
			gateway->push_event(session->handle, &janus_videoroom_plugin, NULL, event, NULL);
			json_decref(event);
			janus_refcount_decrease(&publisher->ref);
		} else {
			JANUS_LOG(LOG_WARN, "Got a slow uplink on a VideoRoom publisher? Weird, because it doesn't receive media...\n");
		}
	} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
		if(uplink) {
			janus_videoroom_subscriber *subscriber = janus_videoroom_session_get_subscriber(session);
			if(subscriber == NULL) {
				janus_refcount_decrease(&session->ref);
				return;
			}
			if(g_atomic_int_get(&subscriber->destroyed)) {
				janus_refcount_decrease(&subscriber->ref);
				janus_refcount_decrease(&session->ref);
				return;
			}
			/* Send an event on the handle to notify the application: it's
			 * up to the application to then choose a policy and enforce it */
			json_t *event = json_object();
			json_object_set_new(event, "videoroom", json_string("slow_link"));
			gateway->push_event(session->handle, &janus_videoroom_plugin, NULL, event, NULL);
			json_decref(event);
		} else {
			JANUS_LOG(LOG_WARN, "Got a slow downlink on a VideoRoom viewer? Weird, because it doesn't send media...\n");
		}
	}
	janus_refcount_decrease(&session->ref);
}

static void janus_videoroom_recorder_create(janus_videoroom_publisher_stream *ps) {
	char filename[255];
	janus_recorder *rc = NULL;
	gint64 now = janus_get_real_time();
	if(ps->publisher && ps->rc == NULL) {
		janus_videoroom_publisher *participant = ps->publisher;
		const char *type = NULL;
		switch(ps->type) {
			case JANUS_VIDEOROOM_MEDIA_AUDIO:
				type = janus_audiocodec_name(ps->acodec);
				break;
			case JANUS_VIDEOROOM_MEDIA_VIDEO:
				type = janus_videocodec_name(ps->vcodec);
				break;
			case JANUS_VIDEOROOM_MEDIA_DATA:
				type = "text";
				break;
			default:
				return;
		}
		janus_rtp_switching_context_reset(&ps->rec_ctx);
		janus_rtp_simulcasting_context_reset(&ps->rec_simctx);
		ps->rec_simctx.substream_target = 2;
		ps->rec_simctx.templayer_target = 2;
		memset(filename, 0, 255);
		if(participant->recording_base) {
			/* Use the filename and path we have been provided */
			g_snprintf(filename, 255, "%s-%s-%d", participant->recording_base,
				janus_videoroom_media_str(ps->type), ps->mindex);
			rc = janus_recorder_create_full(participant->room->rec_dir, type, ps->fmtp, filename);
			if(rc == NULL) {
				JANUS_LOG(LOG_ERR, "Couldn't open a %s recording file for this publisher!\n",
					janus_videoroom_media_str(ps->type));
			}
		} else {
			/* Build a filename */
			g_snprintf(filename, 255, "videoroom-%s-user-%s-%"SCNi64"-%s-%d",
				participant->room_id_str, participant->user_id_str, now,
				janus_videoroom_media_str(ps->type), ps->mindex);
			rc = janus_recorder_create_full(participant->room->rec_dir, type, ps->fmtp, filename);
			if(rc == NULL) {
				JANUS_LOG(LOG_ERR, "Couldn't open an %s recording file for this publisher!\n",
					janus_videoroom_media_str(ps->type));
			}
		}
		/* If the stream has a description, store it in the recording */
		if(ps->description)
			janus_recorder_description(rc, ps->description);
		/* If the video-orientation extension has been negotiated, mark it in the recording */
		if(ps->video_orient_extmap_id > 0)
			janus_recorder_add_extmap(rc, ps->video_orient_extmap_id, JANUS_RTP_EXTMAP_VIDEO_ORIENTATION);
		/* If media is encrypted, mark it in the recording */
		if(ps->type != JANUS_VIDEOROOM_MEDIA_DATA && participant->e2ee)
			janus_recorder_encrypted(rc);
		ps->rc = rc;
	}
}

static void janus_videoroom_recorder_close(janus_videoroom_publisher *participant) {
	GList *temp = participant->streams;
	while(temp) {
		janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
		if(ps->rc) {
			janus_recorder *rc = ps->rc;
			ps->rc = NULL;
			janus_recorder_close(rc);
			JANUS_LOG(LOG_INFO, "Closed %s recording %s\n", janus_videoroom_media_str(ps->type),
				rc->filename ? rc->filename : "??");
			janus_recorder_destroy(rc);
		}
		temp = temp->next;
	}
}

void janus_videoroom_hangup_media(janus_plugin_session *handle) {
	JANUS_LOG(LOG_INFO, "[%s-%p] No WebRTC media anymore; %p %p\n", JANUS_VIDEOROOM_PACKAGE, handle, handle->gateway_handle, handle->plugin_handle);
	if(g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_mutex_lock(&sessions_mutex);
	janus_videoroom_session *session = janus_videoroom_lookup_session(handle);
	if(!session) {
		JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
		janus_mutex_unlock(&sessions_mutex);
		return;
	}
	if(g_atomic_int_get(&session->destroyed)) {
		janus_mutex_unlock(&sessions_mutex);
		return;
	}
	janus_refcount_increase(&session->ref);
	janus_mutex_unlock(&sessions_mutex);
	janus_videoroom_hangup_media_internal(session);
	janus_refcount_decrease(&session->ref);
}

static void janus_videoroom_hangup_media_internal(gpointer session_data) {
	janus_videoroom_session *session = (janus_videoroom_session *)session_data;
	g_atomic_int_set(&session->started, 0);
	if(!g_atomic_int_compare_and_exchange(&session->hangingup, 0, 1)) {
		return;
	}
	g_atomic_int_set(&session->dataready, 0);
	/* Send an event to the browser and tell the PeerConnection is over */
	if(session->participant_type == janus_videoroom_p_type_publisher) {
		/* This publisher just 'unpublished' */
		janus_videoroom_publisher *participant = janus_videoroom_session_get_publisher(session);
		/* Get rid of the recorders, if available */
		janus_mutex_lock(&participant->rec_mutex);
		g_free(participant->recording_base);
		participant->recording_base = NULL;
		janus_videoroom_recorder_close(participant);
		janus_mutex_unlock(&participant->rec_mutex);
		participant->acodec = JANUS_AUDIOCODEC_NONE;
		participant->vcodec = JANUS_VIDEOCODEC_NONE;
		participant->firefox = FALSE;
		participant->e2ee = FALSE;
		/* Get rid of streams */
		janus_mutex_lock(&participant->streams_mutex);
		GList *subscribers = NULL;
		GList *temp = participant->streams;
		while(temp) {
			janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
			/* Close all subscriptions to this stream */
			janus_mutex_lock(&ps->subscribers_mutex);
			GSList *temp2 = ps->subscribers;
			while(temp2) {
				janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)temp2->data;
				temp2 = temp2->next;
				if(ss) {
					/* Take note of the subscriber, so that we can send an updated offer */
					if(ss->type != JANUS_VIDEOROOM_MEDIA_DATA && g_list_find(subscribers, ss->subscriber) == NULL) {
						janus_refcount_increase(&ss->subscriber->ref);
						janus_refcount_increase(&ss->subscriber->session->ref);
						subscribers = g_list_append(subscribers, ss->subscriber);
					}
					/* Remove the subscription (turns the m-line to inactive) */
					janus_videoroom_subscriber_stream_remove(ss, ps, FALSE);
				}
			}
			g_slist_free(ps->subscribers);
			ps->subscribers = NULL;
			janus_rtp_simulcasting_cleanup(&ps->rid_extmap_id, ps->vssrc, ps->rid, &ps->rid_mutex);
			g_free(ps->fmtp);
			ps->fmtp = NULL;
			janus_mutex_unlock(&ps->subscribers_mutex);
			temp = temp->next;
		}
		/* Any subscriber session to update? */
		if(subscribers != NULL) {
			temp = subscribers;
			while(temp) {
				janus_videoroom_subscriber *subscriber = (janus_videoroom_subscriber *)temp->data;
				/* Send (or schedule) a new offer */
				janus_mutex_lock(&subscriber->streams_mutex);
				if(!g_atomic_int_get(&subscriber->answered)) {
					/* We're still waiting for an answer to a previous offer, postpone this */
					g_atomic_int_set(&subscriber->pending_offer, 1);
					janus_mutex_unlock(&subscriber->streams_mutex);
				} else {
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("updated"));
					json_object_set_new(event, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, FALSE, NULL);
					json_t *media_event = NULL;
					if(notify_events && gateway->events_is_enabled())
						media_event = json_deep_copy(media);
					json_object_set_new(event, "streams", media);
					/* Generate a new offer */
					json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
					janus_mutex_unlock(&subscriber->streams_mutex);
					/* How long will the Janus core take to push the event? */
					gint64 start = janus_get_monotonic_time();
					int res = gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, jsep);
					JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
					json_decref(event);
					json_decref(jsep);
					/* Also notify event handlers */
					if(notify_events && gateway->events_is_enabled()) {
						json_t *info = json_object();
						json_object_set_new(info, "event", json_string("updated"));
						json_object_set_new(info, "room", string_ids ?
							json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
						json_object_set_new(info, "streams", media_event);
						json_object_set_new(info, "private_id", json_integer(subscriber->pvt_id));
						gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
					}
				}
				janus_refcount_decrease(&subscriber->session->ref);
				janus_refcount_decrease(&subscriber->ref);
				temp = temp->next;
			}
		}
		g_list_free(subscribers);
		/* Free streams */
		g_list_free(participant->streams);
		participant->streams = NULL;
		g_hash_table_remove_all(participant->streams_byid);
		g_hash_table_remove_all(participant->streams_bymid);
		janus_mutex_unlock(&participant->streams_mutex);
		janus_videoroom_leave_or_unpublish(participant, FALSE, FALSE);
		janus_refcount_decrease(&participant->ref);
	} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
		/* Get rid of subscriber */
		janus_videoroom_subscriber *subscriber = janus_videoroom_session_get_subscriber(session);
		if(subscriber) {
			janus_videoroom_subscriber_print_stat(subscriber);
			subscriber->paused = TRUE;
			subscriber->e2ee = FALSE;
			g_atomic_int_set(&subscriber->answered, 0);
			g_atomic_int_set(&subscriber->pending_offer, 0);
			g_atomic_int_set(&subscriber->pending_restart, 0);
			/* Get rid of streams */
			janus_mutex_lock(&subscriber->streams_mutex);
			GList *temp = subscriber->streams;
			while(temp) {
				janus_videoroom_subscriber_stream *s = (janus_videoroom_subscriber_stream *)temp->data;
				GSList *list = s->publisher_streams;
				while(list) {
					janus_videoroom_publisher_stream *ps = list->data;
					if(ps && ps->publisher != NULL) {
						/* Also notify event handlers */
						if(notify_events && gateway->events_is_enabled()) {
							json_t *info = json_object();
							json_object_set_new(info, "event", json_string("unsubscribed"));
							json_object_set_new(info, "room", string_ids ?
								json_string(ps->publisher->room_id_str) : json_integer(ps->publisher->room_id));
							json_object_set_new(info, "feed",  string_ids ?
								json_string(ps->publisher->user_id_str) : json_integer(ps->publisher->user_id));
							json_object_set_new(info, "mid", json_string(ps->mid));
							gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
						}
					}
					list = list->next;
				}
				janus_videoroom_subscriber_stream_remove(s, NULL, TRUE);
				temp = temp->next;
			}
			/* Free streams */
			g_list_free(subscriber->streams);
			subscriber->streams = NULL;
			g_hash_table_remove_all(subscriber->streams_byid);
			g_hash_table_remove_all(subscriber->streams_bymid);
			janus_mutex_unlock(&subscriber->streams_mutex);
			janus_refcount_decrease(&subscriber->ref);
		}
		/* TODO Should we close the handle as well? */
	}
	g_atomic_int_set(&session->hangingup, 0);
}

static void add_other_publishers_to_event(json_t *event, gpointer user_id_ptr, janus_videoroom *videoroom) {
	/* expected videoroom->mutex is locked */
	janus_videoroom_publisher *publisher = g_hash_table_lookup(videoroom->participants, user_id_ptr);
	if(publisher == NULL || g_atomic_int_get(&publisher->destroyed))
		return;

	json_t *list = json_array(), *attendees = NULL;
	if(videoroom->notify_joining)
		attendees = json_array();
	/* Return a list of all available publishers (those with an SDP available, that is) */
	GHashTableIter iter;
	gpointer value;
	g_hash_table_iter_init(&iter, videoroom->participants);
	while (!g_atomic_int_get(&videoroom->destroyed) && g_hash_table_iter_next(&iter, NULL, &value)) {
		janus_videoroom_publisher *p = value;
		/* is this correct to compare user_id instead of the whole publisher? */
		if(p == publisher || !p->streams || !g_atomic_int_get(&p->session->started)) {
			/* Check if we're also notifying normal joins and not just publishers */
			if(p != publisher && publisher->room->notify_joining) {
				json_t *al = json_object();
				json_object_set_new(al, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
				if(p->display)
					json_object_set_new(al, "display", json_string(p->display));
				json_array_append_new(attendees, al);
			}
			continue;
		}
		json_t *pl = json_object();
		json_object_set_new(pl, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
		if(p->display)
			json_object_set_new(pl, "display", json_string(p->display));
		if(p->dummy)
			json_object_set_new(pl, "dummy", json_true());
		/* Add proper info on all the streams */
		gboolean audio_added = FALSE, video_added = FALSE, talking_found = FALSE, talking = FALSE;
		json_t *media = json_array();
		GList *temp = p->streams;
		while(temp) {
			janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
			json_t *info = json_object();
			json_object_set_new(info, "type", json_string(janus_videoroom_media_str(ps->type)));
			json_object_set_new(info, "mindex", json_integer(ps->mindex));
			json_object_set_new(info, "mid", json_string(ps->mid));
			if(ps->disabled) {
				json_object_set_new(info, "disabled", json_true());
			} else {
				if(ps->description)
					json_object_set_new(info, "description", json_string(ps->description));
				if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
					json_object_set_new(info, "codec", json_string(janus_audiocodec_name(ps->acodec)));
					/* FIXME For backwards compatibility, we need audio_codec in the global info */
					if(!audio_added) {
						audio_added = TRUE;
						json_object_set_new(pl, "audio_codec", json_string(janus_audiocodec_name(ps->acodec)));
					}
					if(ps->acodec == JANUS_AUDIOCODEC_OPUS) {
						if(ps->opusstereo)
							json_object_set_new(info, "stereo", json_true());
						if(ps->opusfec)
							json_object_set_new(info, "fec", json_true());
						if(ps->opusdtx)
							json_object_set_new(info, "dtx", json_true());
					}
					if(ps->audio_level_extmap_id > 0) {
						json_object_set_new(info, "talking", talking ? json_true() : json_false());
						/* FIXME For backwards compatibility, we also need talking in the global info */
						talking_found = TRUE;
						talking |= ps->talking;
					}
				} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
					/* FIXME For backwards compatibility, we need video_codec in the global info */
					json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
					if(!video_added) {
						video_added = TRUE;
						json_object_set_new(pl, "video_codec", json_string(janus_videocodec_name(ps->vcodec)));
					}
					if(ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile != NULL)
						json_object_set_new(info, "h264_profile", json_string(ps->h264_profile));
					else if(ps->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile != NULL)
						json_object_set_new(info, "vp9_profile", json_string(ps->vp9_profile));
					if(ps->simulcast)
						json_object_set_new(info, "simulcast", json_true());
					if(ps->svc)
						json_object_set_new(info, "svc", json_true());
				}
				if(ps->muted)
					json_object_set_new(info, "moderated", json_true());
			}
			json_array_append_new(media, info);
			temp = temp->next;
		}
		json_object_set_new(pl, "streams", media);
		if(talking_found)
			json_object_set_new(pl, "talking", talking ? json_true() : json_false());
		json_array_append_new(list, pl);
	}
	json_object_set_new(event, "publishers", list);
	if(attendees != NULL)
		json_object_set_new(event, "attendees", attendees);
}

/* Thread to handle incoming messages */
static void *janus_videoroom_handler(void *data) {
	JANUS_LOG(LOG_VERB, "Joining VideoRoom handler thread\n");
	janus_videoroom_message *msg = NULL;
	int error_code = 0;
	char error_cause[512];
	json_t *root = NULL;
	while(g_atomic_int_get(&initialized) && !g_atomic_int_get(&stopping)) {
		msg = g_async_queue_pop(messages);
		if(msg == &exit_message)
			break;
		if(msg->handle == NULL) {
			janus_videoroom_message_free(msg);
			continue;
		}
		janus_videoroom *videoroom = NULL;
		janus_videoroom_publisher *participant = NULL;
		janus_videoroom_subscriber *subscriber = NULL;
		janus_mutex_lock(&sessions_mutex);
		janus_videoroom_session *session = janus_videoroom_lookup_session(msg->handle);
		if(!session) {
			janus_mutex_unlock(&sessions_mutex);
			JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
			janus_videoroom_message_free(msg);
			continue;
		}
		if(g_atomic_int_get(&session->destroyed)) {
			janus_mutex_unlock(&sessions_mutex);
			janus_videoroom_message_free(msg);
			continue;
		}
		if(session->participant_type == janus_videoroom_p_type_subscriber) {
			subscriber = janus_videoroom_session_get_subscriber(session);
			if(subscriber == NULL || g_atomic_int_get(&subscriber->destroyed)) {
				if(subscriber != NULL)
					janus_refcount_decrease(&subscriber->ref);
				janus_mutex_unlock(&sessions_mutex);
				JANUS_LOG(LOG_ERR, "Invalid subscriber instance\n");
				error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
				g_snprintf(error_cause, 512, "Invalid subscriber instance");
				goto error;
			}
			if(subscriber->room == NULL) {
				janus_refcount_decrease(&subscriber->ref);
				janus_mutex_unlock(&sessions_mutex);
				JANUS_LOG(LOG_ERR, "No such room\n");
				error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
				g_snprintf(error_cause, 512, "No such room");
				goto error;
			}
		}
		janus_mutex_unlock(&sessions_mutex);
		/* Handle request */
		error_code = 0;
		root = NULL;
		if(msg->message == NULL) {
			if(session->participant_type == janus_videoroom_p_type_subscriber) {
				janus_refcount_decrease(&subscriber->ref);
			}
			JANUS_LOG(LOG_ERR, "No message??\n");
			error_code = JANUS_VIDEOROOM_ERROR_NO_MESSAGE;
			g_snprintf(error_cause, 512, "%s", "No message??");
			goto error;
		}
		root = msg->message;
		/* Get the request first */
		JANUS_VALIDATE_JSON_OBJECT(root, request_parameters,
			error_code, error_cause, TRUE,
			JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
		if(error_code != 0) {
			if(session->participant_type == janus_videoroom_p_type_subscriber) {
				janus_refcount_decrease(&subscriber->ref);
			}
			goto error;
		}
		json_t *request = json_object_get(root, "request");
		const char *request_text = json_string_value(request);
		json_t *event = NULL;
		gpointer add_other_publishers_to_this = NULL;
		gboolean sdp_update = FALSE;
		if(json_object_get(msg->jsep, "update") != NULL)
			sdp_update = json_is_true(json_object_get(msg->jsep, "update"));
		/* 'create' and 'destroy' are handled synchronously: what kind of participant is this session referring to? */
		if(session->participant_type == janus_videoroom_p_type_none) {
			JANUS_LOG(LOG_VERB, "Configuring new participant\n");
			/* Not configured yet, we need to do this now */
			if(strcasecmp(request_text, "join") && strcasecmp(request_text, "joinandconfigure")) {
				JANUS_LOG(LOG_ERR, "Invalid request \"%s\" on unconfigured participant\n", request_text);
				error_code = JANUS_VIDEOROOM_ERROR_JOIN_FIRST;
				g_snprintf(error_cause, 512, "Invalid request on unconfigured participant");
				goto error;
			}
			if(!string_ids) {
				JANUS_VALIDATE_JSON_OBJECT(root, room_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			} else {
				JANUS_VALIDATE_JSON_OBJECT(root, roomstr_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			}
			if(error_code != 0)
				goto error;
			JANUS_VALIDATE_JSON_OBJECT(root, join_parameters,
				error_code, error_cause, TRUE,
				JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
			if(error_code != 0)
				goto error;
			janus_mutex_lock(&rooms_mutex);
			error_code = janus_videoroom_access_room(root, FALSE, TRUE, &videoroom, error_cause, sizeof(error_cause));
			if(error_code != 0) {
				janus_mutex_unlock(&rooms_mutex);
				goto error;
			}
			janus_refcount_increase(&videoroom->ref);
			janus_mutex_unlock(&rooms_mutex);
			janus_mutex_lock(&videoroom->mutex);
			json_t *ptype = json_object_get(root, "ptype");
			const char *ptype_text = json_string_value(ptype);
			if(!strcasecmp(ptype_text, "publisher")) {
				JANUS_LOG(LOG_VERB, "Configuring new publisher\n");
				JANUS_VALIDATE_JSON_OBJECT(root, publisher_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0) {
					janus_mutex_unlock(&videoroom->mutex);
					janus_refcount_decrease(&videoroom->ref);
					goto error;
				}
				if(!string_ids) {
					JANUS_VALIDATE_JSON_OBJECT(root, idopt_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				} else {
					JANUS_VALIDATE_JSON_OBJECT(root, idstropt_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				}
				if(error_code != 0) {
					janus_mutex_unlock(&videoroom->mutex);
					janus_refcount_decrease(&videoroom->ref);
					goto error;
				}
				json_t *descriptions = json_object_get(root, "descriptions");
				if(descriptions != NULL && json_array_size(descriptions) > 0) {
					size_t i = 0;
					for(i=0; i<json_array_size(descriptions); i++) {
						json_t *d = json_array_get(descriptions, i);
						JANUS_VALIDATE_JSON_OBJECT(d, publish_desc_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						if(error_code != 0) {
							janus_mutex_unlock(&videoroom->mutex);
							janus_refcount_decrease(&videoroom->ref);
							goto error;

						}
					}
				}
				/* A token might be required to join */
				if(videoroom->check_allowed) {
					json_t *token = json_object_get(root, "token");
					const char *token_text = token ? json_string_value(token) : NULL;
					if(token_text == NULL || g_hash_table_lookup(videoroom->allowed, token_text) == NULL) {
						janus_mutex_unlock(&videoroom->mutex);
						janus_refcount_decrease(&videoroom->ref);
						JANUS_LOG(LOG_ERR, "Unauthorized (not in the allowed list)\n");
						error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
						g_snprintf(error_cause, 512, "Unauthorized (not in the allowed list)");
						goto error;
					}
				}
				json_t *display = json_object_get(root, "display");
				const char *display_text = display ? json_string_value(display) : NULL;
				guint64 user_id = 0;
				char user_id_num[30], *user_id_str = NULL;
				gboolean user_id_allocated = FALSE;
				json_t *id = json_object_get(root, "id");
				if(id) {
					if(!string_ids) {
						user_id = json_integer_value(id);
						g_snprintf(user_id_num, sizeof(user_id_num), "%"SCNu64, user_id);
						user_id_str = user_id_num;
					} else {
						user_id_str = (char *)json_string_value(id);
					}
					if(g_hash_table_lookup(videoroom->participants,
							string_ids ? (gpointer)user_id_str : (gpointer)&user_id) != NULL) {
						/* User ID already taken */
						janus_mutex_unlock(&videoroom->mutex);
						janus_refcount_decrease(&videoroom->ref);
						error_code = JANUS_VIDEOROOM_ERROR_ID_EXISTS;
						JANUS_LOG(LOG_ERR, "User ID %s already exists\n", user_id_str);
						g_snprintf(error_cause, 512, "User ID %s already exists", user_id_str);
						goto error;
					}
				}
				if(!string_ids) {
					if(user_id == 0) {
						/* Generate a random ID */
						while(user_id == 0) {
							user_id = janus_random_uint64();
							if(g_hash_table_lookup(videoroom->participants, &user_id) != NULL) {
								/* User ID already taken, try another one */
								user_id = 0;
							}
						}
						g_snprintf(user_id_num, sizeof(user_id_num), "%"SCNu64, user_id);
						user_id_str = user_id_num;
					}
					JANUS_LOG(LOG_VERB, "  -- Participant ID: %"SCNu64"\n", user_id);
				} else {
					if(user_id_str == NULL) {
						/* Generate a random ID */
						while(user_id_str == NULL) {
							user_id_str = janus_random_uuid();
							if(g_hash_table_lookup(videoroom->participants, user_id_str) != NULL) {
								/* User ID already taken, try another one */
								g_clear_pointer(&user_id_str, g_free);
							}
						}
						user_id_allocated = TRUE;
					}
					JANUS_LOG(LOG_VERB, "  -- Participant ID: %s\n", user_id_str);
				}
				/* Process the request */
				json_t *bitrate = NULL, *record = NULL, *recfile = NULL,
					*audiocodec = NULL, *videocodec = NULL,
					*user_audio_active_packets = NULL, *user_audio_level_average = NULL, *want_mixed_audio = NULL;
				if(!strcasecmp(request_text, "joinandconfigure")) {
					/* Also configure (or publish a new feed) audio/video/bitrate for this new publisher */
					/* join_parameters were validated earlier. */
					audiocodec = json_object_get(root, "audiocodec");
					videocodec = json_object_get(root, "videocodec");
					bitrate = json_object_get(root, "bitrate");
					record = json_object_get(root, "record");
					recfile = json_object_get(root, "filename");
					want_mixed_audio = json_object_get(root, "audioMix");
				}
				user_audio_active_packets = json_object_get(root, "audio_active_packets");
				user_audio_level_average = json_object_get(root, "audio_level_average");
				janus_videoroom_publisher *publisher = g_malloc0(sizeof(janus_videoroom_publisher));
				publisher->session = session;
				publisher->room_id = videoroom->room_id;
				publisher->room_id_str = videoroom->room_id_str ? g_strdup(videoroom->room_id_str) : NULL;
				publisher->room = videoroom;
				videoroom = NULL;
				publisher->user_id = user_id;
				publisher->user_id_str = user_id_str ? g_strdup(user_id_str) : NULL;
				publisher->display = display_text ? g_strdup(display_text) : NULL;
				publisher->recording_active = FALSE;
				publisher->recording_base = NULL;
				publisher->firefox = FALSE;
				publisher->bitrate = publisher->room->bitrate;
				publisher->subscriptions = NULL;
				publisher->acodec = JANUS_AUDIOCODEC_NONE;
				publisher->vcodec = JANUS_VIDEOCODEC_NONE;
				janus_mutex_init(&publisher->subscribers_mutex);
				janus_mutex_init(&publisher->own_subscriptions_mutex);
				publisher->streams_byid = g_hash_table_new_full(NULL, NULL,
					NULL, (GDestroyNotify)janus_videoroom_publisher_stream_destroy);
				publisher->streams_bymid = g_hash_table_new_full(g_str_hash, g_str_equal,
					(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_publisher_stream_unref);
				janus_mutex_init(&publisher->streams_mutex);
				publisher->remb_startup = 4;
				publisher->remb_latest = 0;
				janus_mutex_init(&publisher->rtp_forwarders_mutex);
				publisher->rtp_forwarders = g_hash_table_new(NULL, NULL);
				publisher->srtp_contexts = g_hash_table_new_full(g_str_hash, g_str_equal, NULL, (GDestroyNotify)janus_videoroom_srtp_context_free);
				publisher->udp_sock = -1;
				publisher->support_multitrack = 0;
				speech_mix_participant_init(publisher->room, &publisher->speech_mix_participant, publisher->display);
				/* Finally, generate a private ID: this is only needed in case the participant
				 * wants to allow the plugin to know which subscriptions belong to them */
				publisher->pvt_id = 0;
				while(publisher->pvt_id == 0) {
					publisher->pvt_id = janus_random_uint32();
					if(g_hash_table_lookup(publisher->room->private_ids, GUINT_TO_POINTER(publisher->pvt_id)) != NULL) {
						/* Private ID already taken, try another one */
						publisher->pvt_id = 0;
					}
				}
				g_atomic_int_set(&publisher->destroyed, 0);
				janus_refcount_init(&publisher->ref, janus_videoroom_publisher_free);
				g_atomic_int_inc(&g_publishers_count);
				/* In case we also wanted to configure */
				if(audiocodec && json_string_value(json_object_get(msg->jsep, "sdp")) != NULL) {
					janus_audiocodec acodec = janus_audiocodec_from_name(json_string_value(audiocodec));
					if(acodec == JANUS_AUDIOCODEC_NONE ||
							(acodec != publisher->room->acodec[0] &&
							acodec != publisher->room->acodec[1] &&
							acodec != publisher->room->acodec[2] &&
							acodec != publisher->room->acodec[3] &&
							acodec != publisher->room->acodec[4])) {
						JANUS_LOG(LOG_ERR, "Participant asked for audio codec '%s', but it's not allowed (room %s, user %s)\n",
							json_string_value(audiocodec), publisher->room_id_str, publisher->user_id_str);
						janus_mutex_unlock(&publisher->room->mutex);
						janus_refcount_decrease(&publisher->room->ref);
						janus_refcount_decrease(&publisher->ref);
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Audio codec unavailable in this room");
						goto error;
					}
					JANUS_LOG(LOG_VERB, "Participant asked for audio codec '%s' (room %s, user %s)\n",
						json_string_value(audiocodec), publisher->room_id_str, publisher->user_id_str);
					publisher->acodec = acodec;
				}
				if(videocodec && json_string_value(json_object_get(msg->jsep, "sdp")) != NULL) {
					/* The publisher would like to use a video codec in particular */
					janus_videocodec vcodec = janus_videocodec_from_name(json_string_value(videocodec));
					if(vcodec == JANUS_VIDEOCODEC_NONE ||
							(vcodec != publisher->room->vcodec[0] &&
							vcodec != publisher->room->vcodec[1] &&
							vcodec != publisher->room->vcodec[2] &&
							vcodec != publisher->room->vcodec[3] &&
							vcodec != publisher->room->vcodec[4])) {
						JANUS_LOG(LOG_ERR, "Participant asked for video codec '%s', but it's not allowed (room %s, user %s)\n",
							json_string_value(videocodec), publisher->room_id_str, publisher->user_id_str);
						janus_mutex_unlock(&publisher->room->mutex);
						janus_refcount_decrease(&publisher->room->ref);
						janus_refcount_decrease(&publisher->ref);
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Video codec unavailable in this room");
						goto error;
					}
					JANUS_LOG(LOG_VERB, "Participant asked for video codec '%s' (room %s, user %s)\n",
						json_string_value(videocodec), publisher->room_id_str, publisher->user_id_str);
					publisher->vcodec = vcodec;
				}
				if(bitrate) {
					publisher->bitrate = json_integer_value(bitrate);
					JANUS_LOG(LOG_VERB, "Setting video bitrate: %"SCNu32" (room %s, user %s)\n",
						publisher->bitrate, publisher->room_id_str, publisher->user_id_str);
				}
				if(record) {
					publisher->recording_active = json_is_true(record);
					JANUS_LOG(LOG_VERB, "Setting record property: %s (room %s, user %s)\n",
						publisher->recording_active ? "true" : "false", publisher->room_id_str, publisher->user_id_str);
				}
				if(recfile) {
					publisher->recording_base = g_strdup(json_string_value(recfile));
					JANUS_LOG(LOG_VERB, "Setting recording basename: %s (room %s, user %s)\n",
						publisher->recording_base, publisher->room_id_str, publisher->user_id_str);
				}
				if(user_audio_active_packets) {
					publisher->user_audio_active_packets = json_integer_value(user_audio_active_packets);
					JANUS_LOG(LOG_VERB, "Setting user audio_active_packets: %d (room %s, user %s)\n",
						publisher->user_audio_active_packets, publisher->room_id_str, publisher->user_id_str);
				}
				if(user_audio_level_average) {
					publisher->user_audio_level_average = json_integer_value(user_audio_level_average);
					JANUS_LOG(LOG_VERB, "Setting user audio_level_average: %d (room %s, user %s)\n",
						publisher->user_audio_level_average, publisher->room_id_str, publisher->user_id_str);
				}
				if(want_mixed_audio) {
					publisher->want_mixed_audio = json_is_true(want_mixed_audio);
					JANUS_LOG(LOG_VERB, "Setting want_mixed_audio property: %s (room %s, user %s)\n",
						publisher->want_mixed_audio ? "true" : "false", publisher->room_id_str, publisher->user_id_str);
				}
				/* Done */
				janus_mutex_lock(&session->mutex);
				/* Make sure the session has not been destroyed in the meanwhile */
				if(g_atomic_int_get(&session->destroyed)) {
					janus_mutex_unlock(&session->mutex);
					janus_mutex_unlock(&publisher->room->mutex);
					janus_refcount_decrease(&publisher->room->ref);
					janus_videoroom_publisher_destroy(publisher);
					JANUS_LOG(LOG_ERR, "Session destroyed, invalidating new publisher\n");
					error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
					g_snprintf(error_cause, 512, "Session destroyed, invalidating new publisher");
					goto error;
				}
				session->participant_type = janus_videoroom_p_type_publisher;
				session->participant = publisher;
				janus_refcount_increase(&publisher->ref);
				g_hash_table_insert(publisher->room->participants,
					string_ids ? (gpointer)g_strdup(publisher->user_id_str) : (gpointer)janus_uint64_dup(publisher->user_id),
					publisher);
				g_hash_table_insert(publisher->room->private_ids, GUINT_TO_POINTER(publisher->pvt_id), publisher);
				janus_mutex_unlock(&session->mutex);
				add_other_publishers_to_this = string_ids ? (gpointer)publisher->user_id_str : (gpointer)publisher->user_id;
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("joined"));
				json_object_set_new(event, "room", string_ids ? json_string(publisher->room->room_id_str) :
					json_integer(publisher->room->room_id));
				json_object_set_new(event, "description", json_string(publisher->room->room_name));
				json_object_set_new(event, "id", string_ids ? json_string(user_id_str) : json_integer(user_id));
				json_object_set_new(event, "private_id", json_integer(publisher->pvt_id));
				if(publisher->user_audio_active_packets)
					json_object_set_new(event, "audio_active_packets", json_integer(publisher->user_audio_active_packets));
				if(publisher->user_audio_level_average)
					json_object_set_new(event, "audio_level_average", json_integer(publisher->user_audio_level_average));
				/* See if we need to notify about a new participant joined the room (by default, we don't). */
				janus_videoroom_participant_joining(publisher);

				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("joined"));
					json_object_set_new(info, "room", string_ids ? json_string(publisher->room->room_id_str) :
					json_integer(publisher->room->room_id));
					json_object_set_new(info, "id", string_ids ? json_string(user_id_str) : json_integer(user_id));
					json_object_set_new(info, "private_id", json_integer(publisher->pvt_id));
					if(publisher->room->check_allowed) {
						const char *token = json_string_value(json_object_get(root, "token"));
						json_object_set_new(info, "token", json_string(token));
					}
					if(display_text != NULL)
						json_object_set_new(info, "display", json_string(display_text));
					if(publisher->user_audio_active_packets)
						json_object_set_new(info, "audio_active_packets", json_integer(publisher->user_audio_active_packets));
					if(publisher->user_audio_level_average)
						json_object_set_new(info, "audio_level_average", json_integer(publisher->user_audio_level_average));
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
				janus_mutex_unlock(&publisher->room->mutex);
				if(user_id_allocated)
					g_free(user_id_str);
			} else if(!strcasecmp(ptype_text, "subscriber")) {
				JANUS_LOG(LOG_VERB, "Configuring new subscriber\n");
				/* This is a new subscriber */
				JANUS_VALIDATE_JSON_OBJECT(root, subscriber_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0) {
					janus_mutex_unlock(&videoroom->mutex);
					janus_refcount_decrease(&videoroom->ref);
					goto error;
				}
				janus_mutex_lock(&sessions_mutex);
				session = janus_videoroom_lookup_session(msg->handle);
				if(!session) {
					janus_mutex_unlock(&sessions_mutex);
					janus_mutex_unlock(&videoroom->mutex);
					janus_refcount_decrease(&videoroom->ref);
					JANUS_LOG(LOG_ERR, "No session associated with this handle...\n");
					janus_videoroom_message_free(msg);
					continue;
				}
				if(g_atomic_int_get(&session->destroyed)) {
					janus_mutex_unlock(&sessions_mutex);
					janus_mutex_unlock(&videoroom->mutex);
					janus_refcount_decrease(&videoroom->ref);
					janus_videoroom_message_free(msg);
					continue;
				}
				janus_mutex_unlock(&sessions_mutex);
				/* Who does this subscription belong to? */
				guint64 feed_id = 0;
				char feed_id_num[30], *feed_id_str = NULL;
				json_t *pvt = json_object_get(root, "private_id");
				guint64 pvt_id = json_integer_value(pvt);
				/* The new way of subscribing is specifying the streams we're interested in */
				json_t *feeds = json_object_get(root, "streams");
				gboolean legacy = FALSE;
				if(feeds == NULL || json_array_size(feeds) == 0) {
					/* For backwards compatibility, we still support the old "feed" property, which means
					 * "subscribe to all the feeds from this publisher" (depending on offer_audio, etc.) */
					if(!string_ids) {
						JANUS_VALIDATE_JSON_OBJECT(root, feed_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					} else {
						JANUS_VALIDATE_JSON_OBJECT(root, feedstr_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					}
					if(error_code != 0) {
						janus_mutex_unlock(&videoroom->mutex);
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					json_t *feed = json_object_get(root, "feed");
					if(!feed) {
						JANUS_LOG(LOG_ERR, "At least one between 'streams' and 'feed' must be specified\n");
						error_code = JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT;
						g_snprintf(error_cause, 512, "At least one between 'streams' and 'feed' must be specified");
						janus_mutex_unlock(&videoroom->mutex);
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					/* Create a fake "streams" array and put the only feed there */
					json_t *m = json_array();
					json_t *s = json_object();
					json_object_set_new(s, "feed", string_ids ? json_string(feed_id_str) : json_integer(feed_id));
					json_array_append_new(m, s);
					json_object_set_new(root, "streams", m);
					feeds = json_object_get(root, "streams");
					legacy = TRUE;
					JANUS_LOG(LOG_WARN, "Deprecated subscriber 'join' API: please start looking into the new one for the future\n");
				}
				json_t *cpc = json_object_get(root, "close_pc");
				gboolean close_pc  = cpc ? json_is_true(cpc) : TRUE;
				/* Make sure all the feeds we're subscribing to exist */
				GList *publishers = NULL;
				size_t i = 0;
				for(i=0; i<json_array_size(feeds); i++) {
					json_t *s = json_array_get(feeds, i);
					JANUS_VALIDATE_JSON_OBJECT(s, subscriber_stream_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					if(error_code != 0) {
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					if(!string_ids) {
						JANUS_VALIDATE_JSON_OBJECT(s, feed_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					} else {
						JANUS_VALIDATE_JSON_OBJECT(s, feedstr_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					}
					if(error_code != 0) {
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					json_t *feed = json_object_get(s, "feed");
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					janus_videoroom_publisher *publisher = g_hash_table_lookup(videoroom->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
					if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
							!g_atomic_int_get(&publisher->session->started)) {
						JANUS_LOG(LOG_ERR, "No such feed (%s)\n", feed_id_str);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such feed (%s)", feed_id_str);
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					const char *mid = json_string_value(json_object_get(s, "mid"));
					if(mid != NULL) {
						/* Check the mid too */
						janus_mutex_lock(&publisher->streams_mutex);
						if(g_hash_table_lookup(publisher->streams_bymid, mid) == NULL) {
							janus_mutex_unlock(&publisher->streams_mutex);
							JANUS_LOG(LOG_ERR, "No such mid '%s' in feed (%s)\n", mid, feed_id_str);
							error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
							g_snprintf(error_cause, 512, "No such mid '%s' in feed (%s)", mid, feed_id_str);
							janus_mutex_unlock(&videoroom->mutex);
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&videoroom->ref);
							goto error;
						}
						janus_mutex_unlock(&publisher->streams_mutex);
					}
					json_t *spatial = json_object_get(s, "spatial_layer");
					json_t *sc_substream = json_object_get(s, "substream");
					if(json_integer_value(spatial) < 0 || json_integer_value(spatial) > 2 ||
							json_integer_value(sc_substream) < 0 || json_integer_value(sc_substream) > 2) {
						JANUS_LOG(LOG_ERR, "Invalid element (substream/spatial_layer should be 0, 1 or 2)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (substream/spatial_layer should be 0, 1 or 2)");
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					json_t *temporal = json_object_get(s, "temporal_layer");
					json_t *sc_temporal = json_object_get(s, "temporal");
					if(json_integer_value(temporal) < 0 || json_integer_value(temporal) > 2 ||
							json_integer_value(sc_temporal) < 0 || json_integer_value(sc_temporal) > 2) {
						JANUS_LOG(LOG_ERR, "Invalid element (temporal/temporal_layer should be 0, 1 or 2)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (temporal/temporal_layer should be 0, 1 or 2)");
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					/* Increase the refcount before unlocking so that nobody can remove and free the publisher in the meantime. */
					janus_refcount_increase(&publisher->ref);
					janus_refcount_increase(&publisher->session->ref);
					publishers = g_list_append(publishers, publisher);
				}
				/* FIXME These properties are only there for backwards compatibility */
				json_t *offer_audio = json_object_get(root, "offer_audio");
				json_t *offer_video = json_object_get(root, "offer_video");
				json_t *offer_data = json_object_get(root, "offer_data");
				janus_videoroom_publisher *owner = NULL;
				/* Let's check if this room requires valid private_id values */
				if(videoroom->require_pvtid || !legacy) {
					/* It does, let's make sure this subscription complies */
					owner = g_hash_table_lookup(videoroom->private_ids, GUINT_TO_POINTER(pvt_id));
					if(pvt_id == 0 || owner == NULL) {
						JANUS_LOG(LOG_ERR, "Unauthorized (this room requires a valid private_id)\n");
						error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
						g_snprintf(error_cause, 512, "Unauthorized (this room requires a valid private_id)");
						janus_mutex_unlock(&videoroom->mutex);
						/* Unref publishers */
						while(publishers) {
							janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&videoroom->ref);
						goto error;
					}
					janus_refcount_increase(&owner->ref);
					janus_refcount_increase(&owner->session->ref);
				}
				janus_mutex_unlock(&videoroom->mutex);
				/* Allocate a new subscriber instance */
				janus_videoroom_subscriber *subscriber = g_malloc0(sizeof(janus_videoroom_subscriber));
				subscriber->session = session;
				subscriber->room_id = videoroom->room_id;
				subscriber->room_id_str = videoroom->room_id_str ? g_strdup(videoroom->room_id_str) : NULL;
				subscriber->room = videoroom;
				videoroom = NULL;
				subscriber->pvt_id = pvt_id;
				subscriber->close_pc = close_pc;
				subscriber->paused = TRUE;	/* We need an explicit start from the stream */
				subscriber->streams_byid = g_hash_table_new_full(NULL, NULL,
					NULL, (GDestroyNotify)janus_videoroom_subscriber_stream_destroy);
				subscriber->streams_bymid = g_hash_table_new_full(g_str_hash, g_str_equal,
					(GDestroyNotify)g_free, (GDestroyNotify)janus_videoroom_subscriber_stream_unref);
				subscriber->want_mixed_audio = !legacy;
				subscriber->have_audiomix_subscribtion = FALSE;
				if(owner) {
					owner->support_multitrack = !legacy;
					subscriber->display = g_strdup(owner->display);
				}
				janus_mutex_init(&subscriber->streams_mutex);
				g_atomic_int_set(&subscriber->destroyed, 0);
				janus_refcount_init(&subscriber->ref, janus_videoroom_subscriber_free);
				g_atomic_int_inc(&g_subscribers_count);

				janus_refcount_increase(&subscriber->ref);
				/* FIXME backwards compatibility */
				gboolean do_audio = offer_audio ? json_is_true(offer_audio) : TRUE;
				gboolean do_video = offer_video ? json_is_true(offer_video) : TRUE;
				gboolean do_data = offer_data ? json_is_true(offer_data) : TRUE;
				/* Initialize the subscriber streams */
				gboolean data_added = FALSE;
				janus_videoroom_subscriber_stream *data_stream = NULL;
				for(i=0; i<json_array_size(feeds); i++) {
					json_t *s = json_array_get(feeds, i);
					json_t *feed = json_object_get(s, "feed");
					guint64 feed_id = 0;
					char *feed_id_str = NULL;
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);

					if (publisher && !strcmp(g_audio_mix_dummy_publisher_name, publisher->display)) {
						subscriber->have_audiomix_subscribtion = TRUE;
						break;
					}
				}


				for(i=0; i<json_array_size(feeds); i++) {
					json_t *s = json_array_get(feeds, i);
					json_t *feed = json_object_get(s, "feed");
					guint64 feed_id = 0;
					char *feed_id_str = NULL;
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);

					if(publisher == NULL) {
						/* TODO We shouldn't let this happen... */
						JANUS_LOG(LOG_WARN, "Skipping feed %s...\n", feed_id_str);
						continue;
					}
					janus_mutex_lock(&publisher->streams_mutex);
					const char *mid = json_string_value(json_object_get(s, "mid"));
					const char *crossrefid = json_string_value(json_object_get(s, "crossrefid"));
					json_t *spatial = json_object_get(s, "spatial_layer");
					json_t *sc_substream = json_object_get(s, "substream");
					json_t *temporal = json_object_get(s, "temporal_layer");
					json_t *sc_temporal = json_object_get(s, "temporal");
					json_t *sc_fallback = json_object_get(s, "fallback");
					json_t *min_delay = json_object_get(s, "min_delay");
					json_t *max_delay = json_object_get(s, "max_delay");
					if(mid) {
						/* Subscribe to a specific mid */
						janus_videoroom_publisher_stream *ps = g_hash_table_lookup(publisher->streams_bymid, mid);
						if(ps == NULL) {
							/* TODO We shouldn't let this happen either... */
							JANUS_LOG(LOG_WARN, "Skipping mid %s in feed %s...\n", mid, feed_id_str);
							janus_mutex_unlock(&publisher->streams_mutex);
							continue;
						}
						if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA && data_added) {
							/* We already have a datachannel m-line, no need for others: just update the subscribers list */
							janus_mutex_lock(&ps->subscribers_mutex);
							if(g_slist_find(ps->subscribers, data_stream) == NULL && g_slist_find(data_stream->publisher_streams, ps) == NULL) {
								ps->subscribers = g_slist_append(ps->subscribers, data_stream);
								data_stream->publisher_streams = g_slist_append(data_stream->publisher_streams, ps);
								/* The two streams reference each other */
								janus_refcount_increase(&data_stream->ref);
								janus_refcount_increase(&ps->ref);
							}
							janus_mutex_unlock(&ps->subscribers_mutex);
							janus_mutex_unlock(&publisher->streams_mutex);
							continue;
						}
						janus_videoroom_subscriber_stream *stream = janus_videoroom_subscriber_stream_add(subscriber,
							ps, crossrefid, legacy, do_audio, do_video, do_data);
						if(stream && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO &&
								(spatial || sc_substream || temporal || sc_temporal || sc_fallback)) {
							/* Override the default spatial/substream/temporal targets */
							int substream_target = sc_substream ? json_integer_value(sc_substream) : -1;
							if(sc_substream && substream_target >= 0 && substream_target <= 2)
								stream->sim_context.substream_target = substream_target;
							if(sc_temporal)
								stream->sim_context.templayer_target = json_integer_value(sc_temporal);
							if(sc_fallback)
								stream->sim_context.drop_trigger = json_integer_value(sc_fallback);
							if(spatial)
								stream->target_spatial_layer = json_integer_value(spatial);
							if(temporal)
								stream->target_temporal_layer = json_integer_value(temporal);
						}
						if(stream && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
							/* Override the playout-delay properties */
							if(min_delay) {
								int16_t md = json_integer_value(min_delay);
								if(md < 0) {
									stream->min_delay = -1;
									stream->max_delay = -1;
								} else {
									stream->min_delay = md;
									if(stream->min_delay > stream->max_delay)
										stream->max_delay = stream->min_delay;
								}
							}
							if(max_delay) {
								int16_t md = json_integer_value(max_delay);
								if(md < 0) {
									stream->min_delay = -1;
									stream->max_delay = -1;
								} else {
									stream->max_delay = md;
									if(stream->max_delay < stream->min_delay)
										stream->min_delay = stream->max_delay;
								}
							}
						}
						if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA) {
							data_added = TRUE;
							data_stream = stream;
						}
					} else {
						/* Subscribe to all streams */
						GList *temp = publisher->streams;
						while(temp) {
							janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
							if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA && data_added) {
								/* We already have a datachannel m-line, no need for others: just update the subscribers list */
								janus_mutex_lock(&ps->subscribers_mutex);
								if(g_slist_find(ps->subscribers, data_stream) == NULL && g_slist_find(data_stream->publisher_streams, ps) == NULL) {
									ps->subscribers = g_slist_append(ps->subscribers, data_stream);
									data_stream->publisher_streams = g_slist_append(data_stream->publisher_streams, ps);
									/* The two streams reference each other */
									janus_refcount_increase(&data_stream->ref);
									janus_refcount_increase(&ps->ref);
								}
								janus_mutex_unlock(&ps->subscribers_mutex);
								temp = temp->next;
								continue;
							}

							// Client with Mixer support subscribes to all streams:
							// Skip audio streams from subscribtion, unless it is a dummy 'audiomix' publisher
							if (subscriber->have_audiomix_subscribtion) {
								if (ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
									if (strcmp(g_audio_mix_dummy_publisher_name, publisher->display)) {
										temp = temp->next;
										continue;
									}
								}
							}

							janus_videoroom_subscriber_stream *stream = janus_videoroom_subscriber_stream_add(subscriber,
								ps, crossrefid, legacy, do_audio, do_video, do_data);
							if(stream && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO &&
									(spatial || sc_substream || temporal || sc_temporal)) {
								/* Override the default spatial/substream/temporal targets */
								int substream_target = sc_substream ? json_integer_value(sc_substream) : -1;
								if(sc_substream && substream_target >= 0 && substream_target <= 2)
									stream->sim_context.substream_target = substream_target;
								if(sc_temporal)
									stream->sim_context.templayer_target = json_integer_value(sc_temporal);
								if(spatial)
									stream->target_spatial_layer = json_integer_value(spatial);
								if(temporal)
									stream->target_temporal_layer = json_integer_value(temporal);
							}
							if(stream && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
								/* Override the playout-delay properties */
								if(min_delay) {
									int16_t md = json_integer_value(min_delay);
									if(md < 0) {
										stream->min_delay = -1;
										stream->max_delay = -1;
									} else {
										stream->min_delay = md;
										if(stream->min_delay > stream->max_delay)
											stream->max_delay = stream->min_delay;
									}
								}
								if(max_delay) {
									int16_t md = json_integer_value(max_delay);
									if(md < 0) {
										stream->min_delay = -1;
										stream->max_delay = -1;
									} else {
										stream->max_delay = md;
										if(stream->max_delay < stream->min_delay)
											stream->min_delay = stream->max_delay;
									}
								}
							}
							if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA) {
								data_added = TRUE;
								data_stream = stream;
							}
							temp = temp->next;
						}
					}
					janus_mutex_unlock(&publisher->streams_mutex);
				}
				/* Make sure we subscribed to at least something */
				if(subscriber->streams == NULL) {
					/* No subscription created? Unref publishers */
					if(owner) {
						janus_refcount_decrease(&owner->session->ref);
						janus_refcount_decrease(&owner->ref);
					}
					while(publishers) {
						janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
						janus_refcount_decrease(&publisher->session->ref);
						janus_refcount_decrease(&publisher->ref);
						publishers = g_list_remove(publishers, publisher);
					}
					JANUS_LOG(LOG_ERR, "Can't offer an SDP with no stream\n");
					error_code = JANUS_VIDEOROOM_ERROR_INVALID_SDP;
					g_snprintf(error_cause, 512, "Can't offer an SDP with no stream");
					janus_refcount_decrease(&subscriber->ref);
					janus_videoroom_subscriber_destroy(subscriber);
					goto error;
				}
				session->participant = subscriber;
				if(owner != NULL) {
					/* Note: we should refcount these subscription-publisher mappings as well */
					janus_mutex_lock(&owner->subscribers_mutex);
					owner->subscriptions = g_slist_append(owner->subscriptions, subscriber);
					janus_mutex_unlock(&owner->subscribers_mutex);
					/* Done adding the subscription, owner is safe to be released */
					janus_refcount_decrease(&owner->session->ref);
					janus_refcount_decrease(&owner->ref);
				}
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("attached"));
				json_object_set_new(event, "room", string_ids ?
					json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				/* If this is a legacy subscription, put the feed ID too */
				if(legacy) {
					json_object_set_new(event, "id", string_ids ? json_string(feed_id_str) : json_integer(feed_id));
					json_object_set_new(event, "warning", json_string("deprecated_api"));
				}
				janus_mutex_lock(&subscriber->streams_mutex);
				json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, legacy, event);
				json_t *media_event = NULL;
				if(notify_events && gateway->events_is_enabled())
					media_event = json_deep_copy(media);
				json_object_set_new(event, "streams", media);
				session->participant_type = janus_videoroom_p_type_subscriber;
				JANUS_LOG(LOG_VERB, "Preparing JSON event as a reply\n");
				/* Negotiate by crafting a new SDP matching the subscriptions */
				json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
				janus_mutex_unlock(&subscriber->streams_mutex);
				/* How long will the Janus core take to push the event? */
				g_atomic_int_set(&session->hangingup, 0);
				gint64 start = janus_get_monotonic_time();
				int res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, jsep);
				JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
				json_decref(event);
				json_decref(jsep);
				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("subscribing"));
					json_object_set_new(info, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(info, "streams", media_event);
					json_object_set_new(info, "private_id", json_integer(pvt_id));
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
				/* Decrease the references we took before */
				while(publishers) {
					janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
					janus_refcount_decrease(&publisher->session->ref);
					janus_refcount_decrease(&publisher->ref);
					publishers = g_list_remove(publishers, publisher);
				}
				janus_refcount_decrease(&subscriber->ref);
				janus_videoroom_message_free(msg);
				continue;
			} else {
				janus_mutex_unlock(&videoroom->mutex);
				janus_refcount_decrease(&videoroom->ref);
				JANUS_LOG(LOG_ERR, "Invalid element (ptype)\n");
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
				g_snprintf(error_cause, 512, "Invalid element (ptype)");
				goto error;
			}
		} else if(session->participant_type == janus_videoroom_p_type_publisher) {
			/* Handle this publisher */
			participant = janus_videoroom_session_get_publisher(session);
			if(participant == NULL) {
				JANUS_LOG(LOG_ERR, "Invalid participant instance\n");
				error_code = JANUS_VIDEOROOM_ERROR_UNKNOWN_ERROR;
				g_snprintf(error_cause, 512, "Invalid participant instance");
				goto error;
			}
			if(participant->room == NULL) {
				janus_refcount_decrease(&participant->ref);
				if(!strcasecmp(request_text, "join") || !strcasecmp(request_text, "joinandconfigure")) {
					JANUS_LOG(LOG_ERR, "Not in a room (create a new handle)\n");
					error_code = JANUS_VIDEOROOM_ERROR_ALREADY_JOINED;
					g_snprintf(error_cause, 512, "Not in a room (create a new handle)");
				} else {
					JANUS_LOG(LOG_ERR, "No such room\n");
					error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
					g_snprintf(error_cause, 512, "No such room");
				}
				goto error;
			}
			if(!strcasecmp(request_text, "join") || !strcasecmp(request_text, "joinandconfigure")) {
				janus_refcount_decrease(&participant->ref);
				JANUS_LOG(LOG_ERR, "Already in as a publisher on this handle\n");
				error_code = JANUS_VIDEOROOM_ERROR_ALREADY_JOINED;
				g_snprintf(error_cause, 512, "Already in as a publisher on this handle");
				goto error;
			} else if(!strcasecmp(request_text, "configure") || !strcasecmp(request_text, "publish")) {
				if(!strcasecmp(request_text, "publish") && g_atomic_int_get(&participant->session->started)) {
					janus_refcount_decrease(&participant->ref);
					JANUS_LOG(LOG_ERR, "Can't publish, already published\n");
					error_code = JANUS_VIDEOROOM_ERROR_ALREADY_PUBLISHED;
					g_snprintf(error_cause, 512, "Can't publish, already published");
					goto error;
				}
				if(participant->kicked) {
					janus_refcount_decrease(&participant->ref);
					JANUS_LOG(LOG_ERR, "Unauthorized, you have been kicked\n");
					error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
					g_snprintf(error_cause, 512, "Unauthorized, you have been kicked");
					goto error;
				}
				/* Configure (or publish a new feed) audio/video/bitrate for this publisher */
				JANUS_VALIDATE_JSON_OBJECT(root, publish_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0) {
					janus_refcount_decrease(&participant->ref);
					goto error;
				}
				json_t *descriptions = json_object_get(root, "descriptions");
				if(descriptions != NULL && json_array_size(descriptions) > 0) {
					size_t i = 0;
					for(i=0; i<json_array_size(descriptions); i++) {
						json_t *d = json_array_get(descriptions, i);
						JANUS_VALIDATE_JSON_OBJECT(d, publish_desc_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						if(error_code != 0) {
							janus_refcount_decrease(&participant->ref);
							goto error;
						}
					}
				}
				json_t *audiocodec = json_object_get(root, "audiocodec");
				json_t *videocodec = json_object_get(root, "videocodec");
				json_t *bitrate = json_object_get(root, "bitrate");
				json_t *record = json_object_get(root, "record");
				json_t *recfile = json_object_get(root, "filename");
				json_t *display = json_object_get(root, "display");
				json_t *update = json_object_get(root, "update");
				json_t *user_audio_active_packets = json_object_get(root, "audio_active_packets");
				json_t *user_audio_level_average = json_object_get(root, "audio_level_average");
				/* Audio, video and data are deprecated properties */
				json_t *audio = json_object_get(root, "audio");
				json_t *video = json_object_get(root, "video");
				json_t *data = json_object_get(root, "data");
				/* We use an array of streams to state the changes we want to make,
				 * were for each stream we specify the 'mid' to impact (e.g., send) */
				json_t *streams = json_object_get(root, "streams");
				if(streams == NULL) {
					/* No streams object, check if the properties have been
					 * provided globally, which is how we handled this
					 * request before: if so, create a new fake streams
					 * array, and move the parsed options there */
					streams = json_array();
					json_t *stream = json_object();
					const char *mid = json_string_value(json_object_get(root, "mid"));
					if(mid != NULL)
						json_object_set_new(stream, "mid", json_string(mid));
					json_t *send = json_object_get(root, "send");
					if(send != NULL)
						json_object_set_new(stream, "send", json_is_true(send) ? json_true() : json_false());
					json_t *keyframe = json_object_get(root, "keyframe");
					if(keyframe != NULL)
						json_object_set_new(stream, "keyframe", json_is_true(keyframe) ? json_true() : json_false());
					json_t *min_delay = json_object_get(root, "min_delay");
					if(min_delay != NULL)
						json_object_set_new(stream, "min_delay", json_integer(json_integer_value(min_delay)));
					json_t *max_delay = json_object_get(root, "max_delay");
					if(max_delay != NULL)
						json_object_set_new(stream, "max_delay", json_integer(json_integer_value(max_delay)));
					json_array_append_new(streams, stream);
					json_object_set_new(root, "streams", streams);
				}
				/* Validate all the streams we need to configure */
				janus_mutex_lock(&participant->streams_mutex);
				size_t i = 0;
				size_t streams_size = json_array_size(streams);
				for(i=0; i<streams_size; i++) {
					json_t *s = json_array_get(streams, i);
					JANUS_VALIDATE_JSON_OBJECT(s, publish_stream_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					if(error_code != 0)
						break;
					const char *mid = json_string_value(json_object_get(s, "mid"));
					if(mid == NULL && streams_size > 1) {
						JANUS_LOG(LOG_ERR, "Invalid element (mid can't be null in a streams array)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (mid can't be null in a streams array)");
						break;
					} else if(mid != NULL && g_hash_table_lookup(participant->streams_bymid, mid) == NULL) {
						JANUS_LOG(LOG_ERR, "No such mid '%s' published\n", mid);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such mid '%s' published", mid);
						break;
					}
					if(mid != NULL) {
						json_object_del(root, "audio");
						audio = NULL;
						json_object_del(root, "video");
						video = NULL;
						json_object_del(root, "data");
						data = NULL;
					}
				}
				if(error_code != 0) {
					janus_mutex_unlock(&participant->streams_mutex);
					janus_refcount_decrease(&participant->ref);
					goto error;
				}
				/* A renegotiation may be taking place */
				gboolean do_update = update ? json_is_true(update) : FALSE;
				if(do_update && !sdp_update) {
					JANUS_LOG(LOG_WARN, "Got an 'update' request, but no SDP update? Ignoring...\n");
					do_update = FALSE;
				}
				/* Check if there's an SDP to take into account */
				if(json_string_value(json_object_get(msg->jsep, "sdp"))) {
					if(audiocodec) {
						/* The participant would like to use an audio codec in particular */
						janus_audiocodec acodec = janus_audiocodec_from_name(json_string_value(audiocodec));
						if(acodec == JANUS_AUDIOCODEC_NONE ||
								(acodec != participant->room->acodec[0] &&
								acodec != participant->room->acodec[1] &&
								acodec != participant->room->acodec[2] &&
								acodec != participant->room->acodec[3] &&
								acodec != participant->room->acodec[4])) {
							JANUS_LOG(LOG_ERR, "Participant asked for audio codec '%s', but it's not allowed (room %s, user %s)\n",
								json_string_value(audiocodec), participant->room_id_str, participant->user_id_str);
							janus_mutex_unlock(&participant->streams_mutex);
							janus_refcount_decrease(&participant->ref);
							error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
							g_snprintf(error_cause, 512, "Audio codec unavailable in this room");
							goto error;
						}
						JANUS_LOG(LOG_VERB, "Participant asked for audio codec '%s' (room %s, user %s)\n",
							json_string_value(audiocodec), participant->room_id_str, participant->user_id_str);
						participant->acodec = acodec;
					}
					if(videocodec) {
						/* The participant would like to use a video codec in particular */
						janus_videocodec vcodec = janus_videocodec_from_name(json_string_value(videocodec));
						if(vcodec == JANUS_VIDEOCODEC_NONE ||
								(vcodec != participant->room->vcodec[0] &&
								vcodec != participant->room->vcodec[1] &&
								vcodec != participant->room->vcodec[2] &&
								vcodec != participant->room->vcodec[3] &&
								vcodec != participant->room->vcodec[4])) {
							JANUS_LOG(LOG_ERR, "Participant asked for video codec '%s', but it's not allowed (room %s, user %s)\n",
								json_string_value(videocodec), participant->room_id_str, participant->user_id_str);
							janus_mutex_unlock(&participant->streams_mutex);
							janus_refcount_decrease(&participant->ref);
							error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
							g_snprintf(error_cause, 512, "Video codec unavailable in this room");
							goto error;
						}
						JANUS_LOG(LOG_VERB, "Participant asked for video codec '%s' (room %s, user %s)\n",
							json_string_value(videocodec), participant->room_id_str, participant->user_id_str);
						participant->vcodec = vcodec;
					}
				}
				/* Enforce the requested changes (if configuring) */
				for(i=0; i<json_array_size(streams); i++) {
					/* Get the stream we need to tweak */
					json_t *s = json_array_get(streams, i);
					/* Check which properties we need to tweak */
					const char *mid = json_string_value(json_object_get(s, "mid"));
					json_t *send = json_object_get(s, "send");
					json_t *keyframe = json_object_get(s, "keyframe");
					json_t *min_delay = json_object_get(s, "min_delay");
					json_t *max_delay = json_object_get(s, "max_delay");
					GList *temp = participant->streams;
					while(temp) {
						janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
						gboolean mid_found = (mid && !strcasecmp(ps->mid, mid));
						if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO && (audio || (send && mid_found))) {
							gboolean audio_active = mid_found ? json_is_true(send) : json_is_true(audio);
							if(!ps->active && !ps->muted && audio_active) {
								/* Audio was just resumed, try resetting the RTP headers for viewers */
								janus_mutex_lock(&ps->subscribers_mutex);
								GSList *slist = ps->subscribers;
								while(slist) {
									janus_videoroom_subscriber_stream *s = (janus_videoroom_subscriber_stream *)slist->data;
									if(s)
										s->context.seq_reset = TRUE;
									slist = slist->next;
								}
								janus_mutex_unlock(&ps->subscribers_mutex);
							}
							ps->active = audio_active;
							JANUS_LOG(LOG_VERB, "Setting audio property (%s): %s (room %s, user %s)\n",
								ps->mid, ps->active ? "true" : "false", participant->room_id_str, participant->user_id_str);
						} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && (video || (send && mid_found))) {
							gboolean video_active = mid_found ? json_is_true(send) : json_is_true(video);
							if(!ps->active && !ps->muted && video_active) {
								/* Video was just resumed, try resetting the RTP headers for viewers */
								janus_mutex_lock(&participant->subscribers_mutex);
								GSList *slist = ps->subscribers;
								while(slist) {
									janus_videoroom_subscriber_stream *s = (janus_videoroom_subscriber_stream *)slist->data;
									if(s)
										s->context.seq_reset = TRUE;
									slist = slist->next;
								}
								janus_mutex_unlock(&participant->subscribers_mutex);
							}
							ps->active = video_active;
							JANUS_LOG(LOG_VERB, "Setting video property (%s): %s (room %s, user %s)\n",
								ps->mid, ps->active ? "true" : "false", participant->room_id_str, participant->user_id_str);
						} else if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA && (data || (send && mid_found))) {
							gboolean data_active = mid_found ? json_is_true(send) : json_is_true(data);
							ps->active = data_active;
							JANUS_LOG(LOG_VERB, "Setting data property (%s): %s (room %s, user %s)\n",
								ps->mid, ps->active ? "true" : "false", participant->room_id_str, participant->user_id_str);
						}
						if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && (mid_found || mid == NULL) &&
								keyframe && json_is_true(keyframe)) {
							/* Send a PLI */
							janus_videoroom_reqpli(ps, "Keyframe request");
						}
						if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && (mid_found || mid == NULL)) {
							if(min_delay) {
								int16_t md = json_integer_value(min_delay);
								if(md < 0) {
									ps->min_delay = -1;
									ps->max_delay = -1;
								} else {
									ps->min_delay = md;
									if(ps->min_delay > ps->max_delay)
										ps->max_delay = ps->min_delay;
								}
							}
							if(max_delay) {
								int16_t md = json_integer_value(max_delay);
								if(md < 0) {
									ps->min_delay = -1;
									ps->max_delay = -1;
								} else {
									ps->max_delay = md;
									if(ps->max_delay < ps->min_delay)
										ps->min_delay = ps->max_delay;
								}
							}
						}
						temp = temp->next;
					}
				}
				janus_mutex_unlock(&participant->streams_mutex);
				if(bitrate) {
					participant->bitrate = json_integer_value(bitrate);
					JANUS_LOG(LOG_VERB, "Setting video bitrate: %"SCNu32" (room %s, user %s)\n",
						participant->bitrate, participant->room_id_str, participant->user_id_str);
					/* Send a new REMB */
					if(g_atomic_int_get(&session->started))
						participant->remb_latest = janus_get_monotonic_time();
					gateway->send_remb(msg->handle, participant->bitrate);
				}
				if(user_audio_active_packets) {
					participant->user_audio_active_packets = json_integer_value(user_audio_active_packets);
					JANUS_LOG(LOG_VERB, "Setting user audio_active_packets: %d (room %s, user %s)\n",
						participant->user_audio_active_packets, participant->room_id_str, participant->user_id_str);
				}
				if(user_audio_level_average) {
					participant->user_audio_level_average = json_integer_value(user_audio_level_average);
					JANUS_LOG(LOG_VERB, "Setting user audio_level_average: %d (room %s, user %s)\n",
						participant->user_audio_level_average, participant->room_id_str, participant->user_id_str);
				}
				gboolean record_locked = FALSE;
				if((record || recfile) && participant->room->lock_record && participant->room->room_secret) {
					JANUS_CHECK_SECRET(participant->room->room_secret, root, "secret", error_code, error_cause,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT, JANUS_VIDEOROOM_ERROR_UNAUTHORIZED);
					if(error_code != 0) {
						/* Wrong secret provided, we'll prevent the recording state from being changed */
						record_locked = TRUE;
					}
				}
				janus_mutex_lock(&participant->rec_mutex);
				gboolean prev_recording_active = participant->recording_active;
				if(record && !record_locked) {
					participant->recording_active = json_is_true(record);
					JANUS_LOG(LOG_VERB, "Setting record property: %s (room %s, user %s)\n",
						participant->recording_active ? "true" : "false", participant->room_id_str, participant->user_id_str);
				}
				if(recfile && !record_locked) {
					participant->recording_base = g_strdup(json_string_value(recfile));
					JANUS_LOG(LOG_VERB, "Setting recording basename: %s (room %s, user %s)\n",
						participant->recording_base, participant->room_id_str, participant->user_id_str);
				}
				/* Do we need to do something with the recordings right now? */
				if(participant->recording_active != prev_recording_active) {
					/* Something changed */
					if(!participant->recording_active) {
						/* Not recording (anymore?) */
						janus_videoroom_recorder_close(participant);
					} else if(participant->recording_active && g_atomic_int_get(&participant->session->started)) {
						/* We've started recording, send a PLI/FIR and go on */
						GList *temp = participant->streams;
						while(temp) {
							janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
							janus_videoroom_recorder_create(ps);
							if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
								/* Send a PLI */
								janus_videoroom_reqpli(ps, "Recording video");
							}
							temp = temp->next;
						}
					}
				}
				janus_mutex_unlock(&participant->rec_mutex);
				if(display) {
					janus_mutex_lock(&participant->room->mutex);
					char *old_display = participant->display;
					char *new_display = g_strdup(json_string_value(display));
					participant->display = new_display;
					if(old_display != NULL) {
						/* The display name changed, notify this */
						json_t *display_event = json_object();
						json_object_set_new(display_event, "videoroom", json_string("event"));
						json_object_set_new(display_event, "id", string_ids ?
							json_string(participant->user_id_str) : json_integer(participant->user_id));
						json_object_set_new(display_event, "display", json_string(participant->display));
						if(participant->room && !g_atomic_int_get(&participant->room->destroyed)) {
							janus_videoroom_notify_participants(participant, display_event, FALSE);
						}
						json_decref(display_event);
					}
					g_free(old_display);
					janus_mutex_unlock(&participant->room->mutex);
				}
				/* Are we updating the description? */
				if(descriptions != NULL && json_array_size(descriptions) > 0 && json_string_value(json_object_get(msg->jsep, "sdp")) == NULL) {
					/* We only do this here if this is an SDP-less configure: in case
					 * a renegotiation is involved, descriptions are updated later */
					gboolean desc_updated = FALSE;
					size_t i = 0;
					janus_mutex_lock(&participant->streams_mutex);
					for(i=0; i<json_array_size(descriptions); i++) {
						json_t *d = json_array_get(descriptions, i);
						const char *d_mid = json_string_value(json_object_get(d, "mid"));
						janus_videoroom_publisher_stream *ps = d_mid ? g_hash_table_lookup(participant->streams_bymid, d_mid) : NULL;
						if(ps != NULL) {
							const char *d_desc = json_string_value(json_object_get(d, "description"));
							if(d_desc) {
								desc_updated = TRUE;
								g_free(ps->description);
								ps->description = g_strdup(d_desc);
							}
						}
					}
					janus_mutex_unlock(&participant->streams_mutex);
					/* If at least a description changed, notify everyone else about the publisher details */
					if(desc_updated)
						janus_videoroom_notify_about_publisher(participant, TRUE);
				}
				/* Done */
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
				json_object_set_new(event, "configured", json_string("ok"));
				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("configured"));
					json_object_set_new(info, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
					json_object_set_new(info, "id", string_ids ? json_string(participant->user_id_str) : json_integer(participant->user_id));
						/* TODO Add info on all the configured stuff here, here */
					json_object_set_new(info, "bitrate", json_integer(participant->bitrate));
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
			} else if(!strcasecmp(request_text, "unpublish")) {
				/* This participant wants to unpublish */
				if(!g_atomic_int_get(&participant->session->started)) {
					janus_refcount_decrease(&participant->ref);
					JANUS_LOG(LOG_ERR, "Can't unpublish, not published\n");
					error_code = JANUS_VIDEOROOM_ERROR_NOT_PUBLISHED;
					g_snprintf(error_cause, 512, "Can't unpublish, not published");
					goto error;
				}
				/* Tell the core to tear down the PeerConnection, hangup_media will do the rest */
				janus_videoroom_hangup_media(session->handle);
				gateway->close_pc(session->handle);
				/* Done */
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
				json_object_set_new(event, "unpublished", json_string("ok"));
			} else if(!strcasecmp(request_text, "leave")) {
				/* Prepare an event to confirm the request */
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(participant->room_id_str) : json_integer(participant->room_id));
				json_object_set_new(event, "leaving", json_string("ok"));
				/* This publisher is leaving, tell everybody */
				janus_videoroom_leave_or_unpublish(participant, TRUE, FALSE);
				/* Done */
				g_atomic_int_set(&session->started, 0);
				//~ session->destroy = TRUE;
			} else {
				janus_refcount_decrease(&participant->ref);
				JANUS_LOG(LOG_ERR, "Unknown request '%s'\n", request_text);
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_REQUEST;
				g_snprintf(error_cause, 512, "Unknown request '%s'", request_text);
				goto error;
			}
			janus_refcount_decrease(&participant->ref);
		} else if(session->participant_type == janus_videoroom_p_type_subscriber) {
			/* Handle this subscriber */
			if(!strcasecmp(request_text, "join")) {
				JANUS_LOG(LOG_ERR, "Already in as a subscriber on this handle\n");
				error_code = JANUS_VIDEOROOM_ERROR_ALREADY_JOINED;
				g_snprintf(error_cause, 512, "Already in as a subscriber on this handle");
				janus_refcount_decrease(&subscriber->ref);
				goto error;
			} else if(!strcasecmp(request_text, "start")) {
				/* Start/restart receiving the publisher streams */
				if(subscriber->paused && msg->jsep == NULL) {
					/* This is just resuming a paused subscription, reset the RTP sequence numbers on all streams */
					GList *temp = subscriber->streams;
					while(temp) {
						janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)temp->data;
						stream->context.seq_reset = TRUE;
						janus_videoroom_publisher_stream *ps = stream->publisher_streams ? stream->publisher_streams->data : NULL;
						if(ps && ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO && ps->publisher && ps->publisher->session) {
							/* Send a PLI */
							janus_videoroom_reqpli(ps, "Subscriber start");
						}
						temp = temp->next;
					}
				}
				subscriber->paused = FALSE;
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "started", json_string("ok"));
			} else if(!strcasecmp(request_text, "subscribe") || !strcasecmp(request_text, "unsubscribe") ||
					!strcasecmp(request_text, "update")) {
				/* Update a subscription by adding and/or removing new streams */
				gboolean update = !strcasecmp(request_text, "update");
				gboolean subscribe = update || !strcasecmp(request_text, "subscribe");
				gboolean unsubscribe = update || !strcasecmp(request_text, "unsubscribe");
				if(unsubscribe)
					JANUS_LOG(LOG_VERB, "Removing subscriber streams\n");
				if(subscribe)
					JANUS_LOG(LOG_VERB, "Adding new subscriber streams\n");
				/* Validate the request first */
				if(update) {
					JANUS_VALIDATE_JSON_OBJECT(root, subscriber_combined_update_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				} else {
					JANUS_VALIDATE_JSON_OBJECT(root, subscriber_update_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				}
				if(error_code != 0) {
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				json_t *sub_feeds = NULL, *unsub_feeds = NULL;
				if(subscribe) {
					sub_feeds = json_object_get(root, update ? "subscribe" : "streams");
					if(sub_feeds && json_array_size(sub_feeds) == 0) {
						JANUS_LOG(LOG_ERR, "Empty subscription list\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Empty subscription list");
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
				}
				if(unsubscribe) {
					unsub_feeds = json_object_get(root, update ? "unsubscribe" : "streams");
					if(unsub_feeds && json_array_size(unsub_feeds) == 0) {
						JANUS_LOG(LOG_ERR, "Empty unsubscription list\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Empty unsubscription list");
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					size_t i = 0;
					for(i=0; i<json_array_size(unsub_feeds); i++) {
						json_t *s = json_array_get(unsub_feeds, i);
						JANUS_VALIDATE_JSON_OBJECT(s, subscriber_remove_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						if(error_code != 0) {
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						if(!string_ids) {
							JANUS_VALIDATE_JSON_OBJECT(s, feedopt_parameters,
								error_code, error_cause, TRUE,
								JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						} else {
							JANUS_VALIDATE_JSON_OBJECT(s, feedstropt_parameters,
								error_code, error_cause, TRUE,
								JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						}
						if(error_code != 0) {
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
					}
				}
				if(update && sub_feeds == NULL && unsub_feeds == NULL) {
					/* We require at least one array when handling an "update" request */
					JANUS_LOG(LOG_ERR, "At least one of either 'subscribe' or 'unsubscribe' must be present\n");
					error_code = JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT;
					g_snprintf(error_cause, 512, "At least one of either 'subscribe' or 'unsubscribe' must be present");
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				/* If we're subscribing, make sure all the feeds we're subscribing to exist */
				GList *publishers = NULL;
				if(subscribe) {
					size_t i = 0;
					for(i=0; i<json_array_size(sub_feeds); i++) {
						json_t *s = json_array_get(sub_feeds, i);
						JANUS_VALIDATE_JSON_OBJECT(s, subscriber_stream_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						if(error_code != 0) {
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						if(!string_ids) {
							JANUS_VALIDATE_JSON_OBJECT(s, feed_parameters,
								error_code, error_cause, TRUE,
								JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						} else {
							JANUS_VALIDATE_JSON_OBJECT(s, feedstr_parameters,
								error_code, error_cause, TRUE,
								JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
						}
						if(error_code != 0) {
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						json_t *feed = json_object_get(s, "feed");
						guint64 feed_id = 0;
						char feed_id_num[30], *feed_id_str = NULL;
						if(!string_ids) {
							feed_id = json_integer_value(feed);
							g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
							feed_id_str = feed_id_num;
						} else {
							feed_id_str = (char *)json_string_value(feed);
						}
						janus_mutex_lock(&subscriber->room->mutex);
						janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
							string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
						janus_mutex_unlock(&subscriber->room->mutex);
						if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
								!g_atomic_int_get(&publisher->session->started)) {
							JANUS_LOG(LOG_ERR, "No such feed (%s)\n", feed_id_str);
							error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
							g_snprintf(error_cause, 512, "No such feed (%s)", feed_id_str);
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						const char *mid = json_string_value(json_object_get(s, "mid"));
						if(mid != NULL) {
							/* Check the mid too */
							janus_mutex_lock(&publisher->streams_mutex);
							if(g_hash_table_lookup(publisher->streams_bymid, mid) == NULL) {
								janus_mutex_unlock(&publisher->streams_mutex);
								JANUS_LOG(LOG_ERR, "No such mid '%s' in feed (%s)\n", mid, feed_id_str);
								error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
								g_snprintf(error_cause, 512, "No such mid '%s' in feed (%s)", mid, feed_id_str);
								/* Unref publishers we may have taken note of so far */
								while(publishers) {
									publisher = (janus_videoroom_publisher *)publishers->data;
									janus_refcount_decrease(&publisher->session->ref);
									janus_refcount_decrease(&publisher->ref);
									publishers = g_list_remove(publishers, publisher);
								}
								janus_refcount_decrease(&subscriber->ref);
								goto error;
							}
							janus_mutex_unlock(&publisher->streams_mutex);
						}
						json_t *spatial = json_object_get(s, "spatial_layer");
						json_t *sc_substream = json_object_get(s, "substream");
						if(json_integer_value(spatial) < 0 || json_integer_value(spatial) > 2 ||
								json_integer_value(sc_substream) < 0 || json_integer_value(sc_substream) > 2) {
							JANUS_LOG(LOG_ERR, "Invalid element (substream/spatial_layer should be 0, 1 or 2)\n");
							error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
							g_snprintf(error_cause, 512, "Invalid value (substream/spatial_layer should be 0, 1 or 2)");
							janus_mutex_unlock(&videoroom->mutex);
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						json_t *temporal = json_object_get(s, "temporal_layer");
						json_t *sc_temporal = json_object_get(s, "temporal");
						if(json_integer_value(temporal) < 0 || json_integer_value(temporal) > 2 ||
								json_integer_value(sc_temporal) < 0 || json_integer_value(sc_temporal) > 2) {
							JANUS_LOG(LOG_ERR, "Invalid element (temporal/temporal_layer should be 0, 1 or 2)\n");
							error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
							g_snprintf(error_cause, 512, "Invalid value (temporal/temporal_layer should be 0, 1 or 2)");
							janus_mutex_unlock(&videoroom->mutex);
							/* Unref publishers we may have taken note of so far */
							while(publishers) {
								publisher = (janus_videoroom_publisher *)publishers->data;
								janus_refcount_decrease(&publisher->session->ref);
								janus_refcount_decrease(&publisher->ref);
								publishers = g_list_remove(publishers, publisher);
							}
							janus_refcount_decrease(&subscriber->ref);
							goto error;
						}
						/* Increase the refcount before unlocking so that nobody can remove and free the publisher in the meantime. */
						janus_refcount_increase(&publisher->ref);
						janus_refcount_increase(&publisher->session->ref);
						publishers = g_list_append(publishers, publisher);
					}
				}
				/* Update the subscription, now: if this is a combined request, always
				 * handle the unsubscribe first, and the subscribe only after that */
				int changes = 0;
				size_t i = 0;
				janus_mutex_lock(&subscriber->streams_mutex);
				if(unsubscribe) {
					/* Remove the specified subscriptions */
					for(i=0; i<json_array_size(unsub_feeds); i++) {
						json_t *s = json_array_get(unsub_feeds, i);
						json_t *feed = json_object_get(s, "feed");
						guint64 feed_id = 0;
						char feed_id_num[30], *feed_id_str = NULL;
						if(!string_ids) {
							feed_id = json_integer_value(feed);
							g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
							feed_id_str = feed_id_num;
						} else {
							feed_id_str = (char *)json_string_value(feed);
						}
						const char *sub_mid = json_string_value(json_object_get(s, "sub_mid"));
						janus_videoroom_subscriber_stream *stream = NULL;
						if(sub_mid) {
							/* A specific subscription mid has been provided */
							stream = g_hash_table_lookup(subscriber->streams_bymid, sub_mid);
							if(stream == NULL) {
								JANUS_LOG(LOG_WARN, "Subscriber stream with mid '%s' not found, not unsubscribing...\n", sub_mid);
								continue;
							}
							janus_videoroom_subscriber_stream_remove(stream, NULL, TRUE);
							changes++;
						} else if(feed_id_str != NULL) {
							janus_mutex_lock(&subscriber->room->mutex);
							janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
								string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
							janus_mutex_unlock(&subscriber->room->mutex);
							if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
									!g_atomic_int_get(&publisher->session->started)) {
								JANUS_LOG(LOG_WARN, "Publisher '%s' not found, not unsubscribing...\n", feed_id_str);
								continue;
							}
							/* Are we unsubscribing from the publisher as a whole or only a single stream? */
							const char *mid = json_string_value(json_object_get(s, "mid"));
							/* Iterate on all subscriptions, and remove those that don't match */
							GList *temp = subscriber->streams;
							while(temp) {
								/* We need more fine grained mechanisms for changing streaming properties */
								janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)temp->data;
								janus_videoroom_publisher_stream *ps = NULL;
								GSList *list = stream->publisher_streams;
								while(list) {
									ps = list->data;
									if(ps == NULL || ps->publisher != publisher) {
										/* Not the publisher we're interested in */
										list = list->next;
										continue;
									}
									if(mid && ps->mid && strcasecmp(ps->mid, mid)) {
										/* Not the mid we're interested in */
										list = list->next;
										continue;
									}
									janus_videoroom_subscriber_stream_remove(stream, ps, TRUE);
									if(stream->type != JANUS_VIDEOROOM_MEDIA_DATA)
										changes++;
									list = list->next;
								}
								temp = temp->next;
							}
						}
					}
				}
				if(subscribe) {
					/* Add streams, or replace existing and inactive ones */
					for(i=0; i<json_array_size(sub_feeds); i++) {
						json_t *s = json_array_get(sub_feeds, i);
						json_t *feed = json_object_get(s, "feed");
						guint64 feed_id = 0;
						char feed_id_num[30], *feed_id_str = NULL;
						if(!string_ids) {
							feed_id = json_integer_value(feed);
							g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
							feed_id_str = feed_id_num;
						} else {
							feed_id_str = (char *)json_string_value(feed);
						}
						janus_mutex_lock(&subscriber->room->mutex);
						janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
							string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
						janus_mutex_unlock(&subscriber->room->mutex);
						if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
								!g_atomic_int_get(&publisher->session->started)) {
							JANUS_LOG(LOG_WARN, "Publisher '%s' not found, not subscribing...\n", feed_id_str);
							continue;
						}
						/* Are we subscribing to this publisher as a whole or only to a single stream? */
						const char *mid = json_string_value(json_object_get(s, "mid"));
						const char *crossrefid = json_string_value(json_object_get(s, "crossrefid"));
						json_t *spatial = json_object_get(s, "spatial_layer");
						json_t *sc_substream = json_object_get(s, "substream");
						json_t *temporal = json_object_get(s, "temporal_layer");
						json_t *sc_temporal = json_object_get(s, "temporal");
						json_t *sc_fallback = json_object_get(s, "fallback");
						json_t *min_delay = json_object_get(s, "min_delay");
						json_t *max_delay = json_object_get(s, "max_delay");
						if(mid != NULL) {
							janus_mutex_lock(&publisher->streams_mutex);
							janus_videoroom_publisher_stream *ps = g_hash_table_lookup(publisher->streams_bymid, mid);
							janus_mutex_unlock(&publisher->streams_mutex);
							if(ps == NULL) {
								JANUS_LOG(LOG_WARN, "No mid '%s' in publisher '%s', not subscribing...\n", mid, feed_id_str);
								continue;
							}
							if(ps->disabled) {
								JANUS_LOG(LOG_WARN, "Skipping disabled m-line...\n");
								continue;
							}
							janus_videoroom_subscriber_stream *stream = janus_videoroom_subscriber_stream_add_or_replace(subscriber, ps, crossrefid);
							if(stream) {
								changes++;
								if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO &&
										(spatial || sc_substream || temporal || sc_temporal)) {
									/* Override the default spatial/substream/temporal targets */
									int substream_target = sc_substream ? json_integer_value(sc_substream) : -1;
									if(sc_substream && substream_target >= 0 && substream_target <= 2)
										stream->sim_context.substream_target = substream_target;
									if(sc_temporal)
										stream->sim_context.templayer_target = json_integer_value(sc_temporal);
									if(sc_fallback)
										stream->sim_context.drop_trigger = json_integer_value(sc_fallback);
									if(spatial)
										stream->target_spatial_layer = json_integer_value(spatial);
									if(temporal)
										stream->target_temporal_layer = json_integer_value(temporal);
								}
								if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
									/* Override the playout-delay properties */
									if(min_delay) {
										int16_t md = json_integer_value(min_delay);
										if(md < 0) {
											stream->min_delay = -1;
											stream->max_delay = -1;
										} else {
											stream->min_delay = md;
											if(stream->min_delay > stream->max_delay)
												stream->max_delay = stream->min_delay;
										}
									}
									if(max_delay) {
										int16_t md = json_integer_value(max_delay);
										if(md < 0) {
											stream->min_delay = -1;
											stream->max_delay = -1;
										} else {
											stream->max_delay = md;
											if(stream->max_delay < stream->min_delay)
												stream->min_delay = stream->max_delay;
										}
									}
								}
							}
						} else {
							janus_mutex_lock(&publisher->streams_mutex);
							GList *temp = publisher->streams;
							while(temp) {
								janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
								if(ps->disabled) {
									JANUS_LOG(LOG_WARN, "Skipping disabled m-line...\n");
									temp = temp->next;
									continue;
								}

								// Client with Mixer support subscribes to all streams:
								// Skip audio streams from subscribtion, unless it is a dummy 'audiomix' publisher
								if (subscriber->have_audiomix_subscribtion) {
									if (ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
										if (strcmp(g_audio_mix_dummy_publisher_name, publisher->display)) {
											temp = temp->next;
											continue;
										}
									}
								}

								janus_videoroom_subscriber_stream *stream = janus_videoroom_subscriber_stream_add_or_replace(subscriber, ps, crossrefid);
								if(stream) {
									changes++;
									if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO &&
											(spatial || sc_substream || temporal || sc_temporal)) {
										/* Override the default spatial/substream/temporal targets */
										int substream_target = sc_substream ? json_integer_value(sc_substream) : -1;
										if(sc_substream && substream_target >= 0 && substream_target <= 2)
											stream->sim_context.substream_target = substream_target;
										if(sc_temporal)
											stream->sim_context.templayer_target = json_integer_value(sc_temporal);
										if(sc_fallback)
											stream->sim_context.drop_trigger = json_integer_value(sc_fallback);
										if(spatial)
											stream->target_spatial_layer = json_integer_value(spatial);
										if(temporal)
											stream->target_temporal_layer = json_integer_value(temporal);
									}
									if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
										/* Override the playout-delay properties */
										if(min_delay) {
											int16_t md = json_integer_value(min_delay);
											if(md < 0) {
												stream->min_delay = -1;
												stream->max_delay = -1;
											} else {
												stream->min_delay = md;
												if(stream->min_delay > stream->max_delay)
													stream->max_delay = stream->min_delay;
											}
										}
										if(max_delay) {
											int16_t md = json_integer_value(max_delay);
											if(md < 0) {
												stream->min_delay = -1;
												stream->max_delay = -1;
											} else {
												stream->max_delay = md;
												if(stream->max_delay < stream->min_delay)
													stream->min_delay = stream->max_delay;
											}
										}
									}
								}
								temp = temp->next;
							}
							janus_mutex_unlock(&publisher->streams_mutex);
						}
					}
				}
				/* We're done: check if this resulted in any actual change */
				if(changes == 0) {
					janus_mutex_unlock(&subscriber->streams_mutex);
					/* Nothing changed, just ack and don't do anything else */
					JANUS_LOG(LOG_VERB, "No change made, skipping renegotiation\n");
					event = json_object();
					json_object_set_new(event, "videoroom", json_string("updated"));
					json_object_set_new(event, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					/* How long will the Janus core take to push the event? */
					gint64 start = janus_get_monotonic_time();
					int res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, NULL);
					JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
					json_decref(event);
					/* Decrease the references we took before */
					while(publishers) {
						janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
						janus_refcount_decrease(&publisher->session->ref);
						janus_refcount_decrease(&publisher->ref);
						publishers = g_list_remove(publishers, publisher);
					}
					janus_refcount_decrease(&subscriber->ref);
					/* Done */
					janus_videoroom_message_free(msg);
					continue;
				}
				if(!g_atomic_int_get(&subscriber->answered)) {
					/* We're still waiting for an answer to a previous offer, postpone this */
					g_atomic_int_set(&subscriber->pending_offer, 1);
					janus_mutex_unlock(&subscriber->streams_mutex);
					JANUS_LOG(LOG_VERB, "Post-poning new offer, waiting for previous answer\n");
					/* Send a temporary event */
					event = json_object();
					json_object_set_new(event, "videoroom", json_string("updating"));
					json_object_set_new(event, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, NULL);
					json_decref(event);
					/* Decrease the references we took before, if any */
					while(publishers) {
						janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
						janus_refcount_decrease(&publisher->session->ref);
						janus_refcount_decrease(&publisher->ref);
						publishers = g_list_remove(publishers, publisher);
					}
					janus_refcount_decrease(&subscriber->ref);
					janus_videoroom_message_free(msg);
					continue;
				}
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("updated"));
				json_object_set_new(event, "room", string_ids ?
					json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, FALSE, NULL);
				json_t *media_event = NULL;
				if(notify_events && gateway->events_is_enabled())
					media_event = json_deep_copy(media);
				json_object_set_new(event, "streams", media);
				/* Generate a new offer */
				json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
				janus_mutex_unlock(&subscriber->streams_mutex);
				/* How long will the Janus core take to push the event? */
				gint64 start = janus_get_monotonic_time();
				int res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, jsep);
				JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
				json_decref(event);
				json_decref(jsep);
				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("updated"));
					json_object_set_new(info, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(info, "streams", media_event);
					json_object_set_new(info, "private_id", json_integer(subscriber->pvt_id));
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
				/* Decrease the references we took before, if any */
				while(publishers) {
					janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
					janus_refcount_decrease(&publisher->session->ref);
					janus_refcount_decrease(&publisher->ref);
					publishers = g_list_remove(publishers, publisher);
				}
				/* Done */
				janus_refcount_decrease(&subscriber->ref);
				janus_videoroom_message_free(msg);
				continue;
			} else if(!strcasecmp(request_text, "configure")) {
				JANUS_VALIDATE_JSON_OBJECT(root, configure_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0) {
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				if(subscriber->kicked) {
					JANUS_LOG(LOG_ERR, "Unauthorized, you have been kicked\n");
					error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
					g_snprintf(error_cause, 512, "Unauthorized, you have been kicked");
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				json_t *restart = json_object_get(root, "restart");
				json_t *update = json_object_get(root, "update");
				/* Audio, video and data are deprecated properties */
				json_t *audio = json_object_get(root, "audio");
				json_t *video = json_object_get(root, "video");
				json_t *data = json_object_get(root, "data");
				/* We use an array of streams to state the changes we want to make,
				 * were for each stream we specify the 'mid' to impact (e.g., send) */
				json_t *streams = json_object_get(root, "streams");
				if(streams == NULL) {
					/* No streams object, check if the properties have been
					 * provided globally, which is how we handled this
					 * request before: if so, create a new fake streams
					 * array, and move the parsed options there */
					streams = json_array();
					json_t *stream = json_object();
					const char *mid = json_string_value(json_object_get(root, "mid"));
					if(mid != NULL)
						json_object_set_new(stream, "mid", json_string(mid));
					json_t *send = json_object_get(root, "send");
					if(send != NULL)
						json_object_set_new(stream, "send", json_is_true(send) ? json_true() : json_false());
					json_t *spatial = json_object_get(root, "spatial_layer");
					if(spatial != NULL)
						json_object_set_new(stream, "spatial_layer", json_integer(json_integer_value(spatial)));
					json_t *sc_substream = json_object_get(root, "substream");
					if(sc_substream != NULL)
						json_object_set_new(stream, "substream", json_integer(json_integer_value(sc_substream)));
					json_t *temporal = json_object_get(root, "temporal_layer");
					if(temporal != NULL)
						json_object_set_new(stream, "temporal_layer", json_integer(json_integer_value(temporal)));
					json_t *sc_temporal = json_object_get(root, "temporal");
					if(sc_temporal != NULL)
						json_object_set_new(stream, "temporal", json_integer(json_integer_value(sc_temporal)));
					json_t *sc_fallback = json_object_get(root, "fallback");
					if(sc_fallback != NULL)
						json_object_set_new(stream, "fallback", json_integer(json_integer_value(sc_fallback)));
					json_t *min_delay = json_object_get(root, "min_delay");
					if(min_delay != NULL)
						json_object_set_new(stream, "min_delay", json_integer(json_integer_value(min_delay)));
					json_t *max_delay = json_object_get(root, "max_delay");
					if(max_delay != NULL)
						json_object_set_new(stream, "max_delay", json_integer(json_integer_value(max_delay)));
					json_array_append_new(streams, stream);
					json_object_set_new(root, "streams", streams);
				}
				/* Validate all the streams we need to configure */
				janus_mutex_lock(&subscriber->streams_mutex);
				size_t i = 0;
				size_t streams_size = json_array_size(streams);
				for(i=0; i<streams_size; i++) {
					json_t *s = json_array_get(streams, i);
					JANUS_VALIDATE_JSON_OBJECT(s, configure_stream_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					if(error_code != 0)
						break;
					const char *mid = json_string_value(json_object_get(s, "mid"));
					if(mid == NULL && streams_size > 1) {
						JANUS_LOG(LOG_ERR, "Invalid element (mid can't be null in a streams array)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (mid can't be null in a streams array)");
						break;
					} else if(mid != NULL && g_hash_table_lookup(subscriber->streams_bymid, mid) == NULL) {
						JANUS_LOG(LOG_ERR, "No such mid '%s' in subscription\n", mid);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such mid '%s' in subscription", mid);
						break;
					}
					if(mid != NULL) {
						json_object_del(root, "audio");
						audio = NULL;
						json_object_del(root, "video");
						video = NULL;
						json_object_del(root, "data");
						data = NULL;
					}
					json_t *spatial = json_object_get(s, "spatial_layer");
					json_t *sc_substream = json_object_get(s, "substream");
					json_t *temporal = json_object_get(s, "temporal_layer");
					json_t *sc_temporal = json_object_get(s, "temporal");
					if(json_integer_value(spatial) < 0 || json_integer_value(spatial) > 2 ||
							json_integer_value(sc_substream) < 0 || json_integer_value(sc_substream) > 2) {
						JANUS_LOG(LOG_ERR, "Invalid element (substream/spatial_layer should be 0, 1 or 2)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (substream/spatial_layer should be 0, 1 or 2)");
						break;
					}
					if(json_integer_value(temporal) < 0 || json_integer_value(temporal) > 2 ||
							json_integer_value(sc_temporal) < 0 || json_integer_value(sc_temporal) > 2) {
						JANUS_LOG(LOG_ERR, "Invalid element (temporal/temporal_layer should be 0, 1 or 2)\n");
						error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
						g_snprintf(error_cause, 512, "Invalid value (temporal/temporal_layer should be 0, 1 or 2)");
						break;
					}
				}
				if(error_code != 0) {
					janus_mutex_unlock(&subscriber->streams_mutex);
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				/* Enforce the requested changes */
				for(i=0; i<json_array_size(streams); i++) {
					/* Get the stream we need to tweak */
					json_t *s = json_array_get(streams, i);
					/* Check which properties we need to tweak */
					const char *mid = json_string_value(json_object_get(s, "mid"));
					json_t *send = json_object_get(s, "send");
					json_t *spatial = json_object_get(s, "spatial_layer");
					json_t *sc_substream = json_object_get(s, "substream");
					json_t *temporal = json_object_get(s, "temporal_layer");
					json_t *sc_temporal = json_object_get(s, "temporal");
					json_t *sc_fallback = json_object_get(s, "fallback");
					json_t *min_delay = json_object_get(s, "min_delay");
					json_t *max_delay = json_object_get(s, "max_delay");
					GList *temp = subscriber->streams;
					while(temp) {
						/* We need more fine grained mechanisms for changing streaming properties */
						janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)temp->data;
						janus_videoroom_publisher_stream *ps = stream->publisher_streams ? stream->publisher_streams->data : NULL;
						if(audio && stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
							gboolean oldaudio = stream->send;
							gboolean newaudio = json_is_true(audio);
							if(!oldaudio && newaudio) {
								/* Audio just resumed, reset the RTP sequence numbers */
								stream->context.seq_reset = TRUE;
							}
							stream->send = newaudio;
							if (!stream->send) {
								char * json_text = json_dumps(root, JSON_INDENT(3) | JSON_PRESERVE_ORDER);
								JANUS_LOG(LOG_ERR, "audio stream %p send disabled due to command %s\n", stream, json_text);
								free(json_text);
							}
						}
						if(video && stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
							gboolean oldvideo = stream->send;
							gboolean newvideo = json_is_true(video);
							if(!oldvideo && newvideo) {
								/* Video just resumed, reset the RTP sequence numbers */
								stream->context.seq_reset = TRUE;
							}
							stream->send = newvideo;
							if(newvideo) {
								/* Send a PLI */
								janus_videoroom_reqpli(ps, "Restoring video for subscriber");
							}
							if (!stream->send) {
								char * json_text = json_dumps(root, JSON_INDENT(3) | JSON_PRESERVE_ORDER);
								JANUS_LOG(LOG_ERR, "video stream %p send disabled due to command %s\n", stream, json_text);
								free(json_text);
							}

						}
						if(data && stream->type == JANUS_VIDEOROOM_MEDIA_DATA) {
							stream->send = json_is_true(data);
							if (!stream->send) {
								char * json_text = json_dumps(root, JSON_INDENT(3) | JSON_PRESERVE_ORDER);
								JANUS_LOG(LOG_ERR, "data stream %p send disabled due to command %s\n", stream, json_text);
								free(json_text);
							}
						}
						/* Let's also see if this is the right mid */
						if(mid && strcasecmp(stream->mid, mid)) {
							temp = temp->next;
							continue;
						}
						if(send) {
							gboolean oldsend = stream->send;
							gboolean newsend = json_is_true(send);
							if(!oldsend && newsend) {
								/* Medium just resumed, reset the RTP sequence numbers */
								stream->context.seq_reset = TRUE;
							}
							stream->send = json_is_true(send);
							if(newsend) {
								/* Send a PLI */
								janus_videoroom_reqpli(ps, "Restoring video for subscriber");
							}
							if (!stream->send) {
								char * json_text = json_dumps(root, JSON_INDENT(3) | JSON_PRESERVE_ORDER);
								JANUS_LOG(LOG_INFO, "stream %p send disabled due to command %s\n", stream, json_text);
								free(json_text);
							}

						}
						/* Next properties are for video only */
						if(stream->type != JANUS_VIDEOROOM_MEDIA_VIDEO) {
							temp = temp->next;
							continue;
						}
						/* Check if a simulcasting-related request is involved */
						if(ps && ps->simulcast) {
							int substream_target = sc_substream ? json_integer_value(sc_substream) : -1;
							if(sc_substream && substream_target >= 0 && substream_target <= 2) {
								stream->sim_context.substream_target = substream_target;
								JANUS_LOG(LOG_VERB, "Setting video SSRC to let through (simulcast): %"SCNu32" (index %d, was %d)\n",
									ps->vssrc[stream->sim_context.substream_target],
									stream->sim_context.substream_target,
									stream->sim_context.substream);
								if(stream->sim_context.substream_target == stream->sim_context.substream) {
									/* No need to do anything, we're already getting the right substream, so notify the user */
									json_t *event = json_object();
									json_object_set_new(event, "videoroom", json_string("event"));
									json_object_set_new(event, "room", string_ids ?
										json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
									json_object_set_new(event, "mid", json_string(stream->mid));
									json_object_set_new(event, "substream", json_integer(stream->sim_context.substream));
									gateway->push_event(msg->handle, &janus_videoroom_plugin, NULL, event, NULL);
									json_decref(event);
								} else {
									/* Send a PLI */
									janus_videoroom_reqpli(ps, "Simulcasting substream change");
								}
							}
							if(ps->vcodec == JANUS_VIDEOCODEC_VP8 && ps->simulcast && sc_temporal) {
								stream->sim_context.templayer_target = json_integer_value(sc_temporal);
								JANUS_LOG(LOG_VERB, "Setting video temporal layer to let through (simulcast): %d (was %d)\n",
									stream->sim_context.templayer_target, stream->sim_context.templayer);
								if(stream->sim_context.templayer_target == stream->sim_context.templayer) {
									/* No need to do anything, we're already getting the right temporal, so notify the user */
									json_t *event = json_object();
									json_object_set_new(event, "videoroom", json_string("event"));
									json_object_set_new(event, "room", string_ids ?
										json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
									json_object_set_new(event, "mid", json_string(stream->mid));
									json_object_set_new(event, "temporal", json_integer(stream->sim_context.templayer));
									gateway->push_event(msg->handle, &janus_videoroom_plugin, NULL, event, NULL);
									json_decref(event);
								} else {
									/* Send a PLI */
									janus_videoroom_reqpli(ps, "Simulcasting temporal layer change");
								}
							}
							if(sc_fallback) {
								stream->sim_context.drop_trigger = json_integer_value(sc_fallback);
							}
						} else if(ps && ps->svc) {
							/* Also check if the viewer is trying to configure a layer change */
							if(spatial) {
								int spatial_layer = json_integer_value(spatial);
								if(spatial_layer > 1) {
									JANUS_LOG(LOG_WARN, "Spatial layer higher than 1, will probably be ignored\n");
								}
								if(spatial_layer == stream->spatial_layer) {
									/* No need to do anything, we're already getting the right spatial layer, so notify the user */
									json_t *event = json_object();
									json_object_set_new(event, "videoroom", json_string("event"));
									json_object_set_new(event, "room", string_ids ?
										json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
									json_object_set_new(event, "mid", json_string(stream->mid));
									json_object_set_new(event, "spatial_layer", json_integer(stream->spatial_layer));
									gateway->push_event(msg->handle, &janus_videoroom_plugin, NULL, event, NULL);
									json_decref(event);
								} else if(spatial_layer != stream->target_spatial_layer) {
									/* Send a PLI to the new RTP forward publisher */
									janus_videoroom_reqpli(ps, "Need to downscale spatially");
								}
								stream->target_spatial_layer = spatial_layer;
							}
							if(temporal) {
								int temporal_layer = json_integer_value(temporal);
								if(temporal_layer > 2) {
									JANUS_LOG(LOG_WARN, "Temporal layer higher than 2, will probably be ignored\n");
								}
								if(temporal_layer == stream->temporal_layer) {
									/* No need to do anything, we're already getting the right temporal layer, so notify the user */
									json_t *event = json_object();
									json_object_set_new(event, "videoroom", json_string("event"));
									json_object_set_new(event, "room", string_ids ?
										json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
									json_object_set_new(event, "mid", json_string(stream->mid));
									json_object_set_new(event, "temporal_layer", json_integer(stream->temporal_layer));
									gateway->push_event(msg->handle, &janus_videoroom_plugin, NULL, event, NULL);
									json_decref(event);
								}
								stream->target_temporal_layer = temporal_layer;
							}
						}
						if(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
							if(min_delay) {
								int16_t md = json_integer_value(min_delay);
								if(md < 0) {
									stream->min_delay = -1;
									stream->max_delay = -1;
								} else {
									stream->min_delay = md;
									if(stream->min_delay > stream->max_delay)
										stream->max_delay = stream->min_delay;
								}
							}
							if(max_delay) {
								int16_t md = json_integer_value(max_delay);
								if(md < 0) {
									stream->min_delay = -1;
									stream->max_delay = -1;
								} else {
									stream->max_delay = md;
									if(stream->max_delay < stream->min_delay)
										stream->min_delay = stream->max_delay;
								}
							}
						}
						temp = temp->next;
					}
				}
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "configured", json_string("ok"));
				/* The user may be interested in an ICE restart */
				gboolean do_restart = restart ? json_is_true(restart) : FALSE;
				gboolean do_update = update ? json_is_true(update) : FALSE;
				if(sdp_update || do_restart || do_update) {
					/* Negotiate by sending the selected publisher SDP back, and/or force an ICE restart */
					if(!g_atomic_int_get(&subscriber->answered)) {
						/* We're still waiting for an answer to a previous offer, postpone this */
						g_atomic_int_set(&subscriber->pending_offer, 1);
						g_atomic_int_set(&subscriber->pending_restart, 1);
						janus_mutex_unlock(&subscriber->streams_mutex);
						JANUS_LOG(LOG_VERB, "Post-poning new ICE restart offer, waiting for previous answer\n");
						janus_videoroom_message_free(msg);
						continue;
					}
					json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
					janus_mutex_unlock(&subscriber->streams_mutex);
					if(do_restart)
						json_object_set_new(jsep, "restart", json_true());
					/* How long will the Janus core take to push the event? */
					gint64 start = janus_get_monotonic_time();
					int res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, jsep);
					JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
					json_decref(event);
					json_decref(jsep);
					/* Done */
					janus_videoroom_message_free(msg);
					continue;
				}
				janus_mutex_unlock(&subscriber->streams_mutex);
			} else if(!strcasecmp(request_text, "pause")) {
				/* Stop receiving the publisher streams for a while */
				subscriber->paused = TRUE;
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "paused", json_string("ok"));
			} else if(!strcasecmp(request_text, "switch")) {
				/* This subscriber wants to switch to a different publisher */
				JANUS_VALIDATE_JSON_OBJECT(root, switch_parameters,
					error_code, error_cause, TRUE,
					JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
				if(error_code != 0) {
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				if(!subscriber->room || g_atomic_int_get(&subscriber->room->destroyed)) {
					JANUS_LOG(LOG_ERR, "Room Destroyed \n");
					error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
					g_snprintf(error_cause, 512, "No such room ");
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				if(g_atomic_int_get(&subscriber->destroyed)) {
					JANUS_LOG(LOG_ERR, "Room Destroyed (%"SCNu64")\n", subscriber->room_id);
					error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
					g_snprintf(error_cause, 512, "No such room (%"SCNu64")", subscriber->room_id);
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				/* While the legacy way of switching by just providing a feed ID is
				 * still supported (at least for now), it isn't flexible enough: the
				 * new proper way of doing that is providing the list of changes that
				 * need to be done, in terms of which stream to switch to, and which
				 * subscription mid to attach it to. This allows for partial switches
				 * (e.g., change the second video source to Bob's camera), while the
				 * old approach simply forces a single publisher as the new source. */
				json_t *feeds = json_object_get(root, "streams");
				json_t *feed = json_object_get(root, "feed");
				GList *publishers = NULL;
				if(feeds == NULL || json_array_size(feeds) == 0) {
					/* For backwards compatibility, we still support the old "feed" property, which means
					 * "switch to all the feeds from this publisher" (much less sophisticated, though) */
					guint64 feed_id = 0;
					char feed_id_num[30], *feed_id_str = NULL;
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					if(feed_id_str == NULL) {
						JANUS_LOG(LOG_ERR, "At least one between 'streams' and 'feed' must be specified\n");
						error_code = JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT;
						g_snprintf(error_cause, 512, "At least one between 'streams' and 'feed' must be specified");
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					janus_mutex_lock(&subscriber->room->mutex);
					janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
					janus_mutex_unlock(&subscriber->room->mutex);
					if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
							!g_atomic_int_get(&publisher->session->started)) {
						JANUS_LOG(LOG_ERR, "No such feed (%s)\n", feed_id_str);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such feed (%s)", feed_id_str);
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					/* Create a fake "streams" list out of this publisher */
					feeds = json_array();
					json_object_set_new(root, "streams", feeds);
					janus_refcount_increase(&publisher->ref);
					GList *temp = publisher->streams, *touched_already = NULL;
					while(temp) {
						janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
						/* Look for a subscriber stream compatible with this publisher stream */
						janus_videoroom_subscriber_stream *stream = NULL;
						GList *temp2 = subscriber->streams;
						while(temp2) {
							stream = (janus_videoroom_subscriber_stream *)temp->data;
							if(stream->type == ps->type && !g_list_find(touched_already, stream) &&
									((stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO && stream->acodec == ps->acodec) ||
									(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && stream->vcodec == ps->vcodec))) {
								/* This streams looks right */
								touched_already = g_list_append(touched_already, stream);
								json_t *s = json_object();
								json_object_set_new(s, "feed",  string_ids ?
									json_string(publisher->user_id_str) : json_integer(publisher->user_id));
								json_object_set_new(s, "mid", json_string(ps->mid));
								json_object_set_new(s, "sub_mid", json_string(stream->mid));
								json_array_append_new(feeds, s);
							} else {
								JANUS_LOG(LOG_WARN, "Skipping %"SCNu64" stream '%s' legacy switch: no compliant subscriber stream\n",
									publisher->user_id, ps->mid);
							}
							temp2 = temp2->next;
						}
						temp = temp->next;
					}
					g_list_free(touched_already);
					janus_refcount_decrease(&publisher->ref);
					/* Take note of the fact this is a legacy request */
					JANUS_LOG(LOG_WARN, "Deprecated VideoRoom 'switch' API: please start looking into the new one for the future\n");
				}
				/* If we got here, we have a feeds list: make sure we have everything we need */
				if(json_array_size(feeds) == 0) {
					JANUS_LOG(LOG_ERR, "Empty switch list\n");
					error_code = JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT;
					g_snprintf(error_cause, 512, "Empty switch list");
					janus_refcount_decrease(&subscriber->ref);
					goto error;
				}
				/* Make sure all the feeds we're subscribing to exist */
				size_t i = 0;
				for(i=0; i<json_array_size(feeds); i++) {
					json_t *s = json_array_get(feeds, i);
					JANUS_VALIDATE_JSON_OBJECT(s, switch_update_parameters,
						error_code, error_cause, TRUE,
						JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					if(error_code != 0) {
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					if(!string_ids) {
						JANUS_VALIDATE_JSON_OBJECT(s, feed_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					} else {
						JANUS_VALIDATE_JSON_OBJECT(s, feedstr_parameters,
							error_code, error_cause, TRUE,
							JANUS_VIDEOROOM_ERROR_MISSING_ELEMENT, JANUS_VIDEOROOM_ERROR_INVALID_ELEMENT);
					}
					if(error_code != 0) {
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					/* Look for the publisher stream to switch to */
					json_t *feed = json_object_get(s, "feed");
					guint64 feed_id = 0;
					char feed_id_num[30], *feed_id_str = NULL;
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					janus_mutex_lock(&subscriber->room->mutex);
					janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
					janus_mutex_unlock(&subscriber->room->mutex);
					if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
							!g_atomic_int_get(&publisher->session->started)) {
						JANUS_LOG(LOG_ERR, "No such feed (%s)\n", feed_id_str);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such feed (%s)", feed_id_str);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					const char *mid = json_string_value(json_object_get(s, "mid"));
					/* Check the mid too */
					janus_mutex_lock(&publisher->streams_mutex);
					if(g_hash_table_lookup(publisher->streams_bymid, mid) == NULL) {
						janus_mutex_unlock(&publisher->streams_mutex);
						JANUS_LOG(LOG_ERR, "No such mid '%s' in feed (%s)\n", mid, feed_id_str);
						error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_FEED;
						g_snprintf(error_cause, 512, "No such mid '%s' in feed (%s)", mid, feed_id_str);
						/* Unref publishers we may have taken note of so far */
						while(publishers) {
							publisher = (janus_videoroom_publisher *)publishers->data;
							janus_refcount_decrease(&publisher->session->ref);
							janus_refcount_decrease(&publisher->ref);
							publishers = g_list_remove(publishers, publisher);
						}
						janus_refcount_decrease(&subscriber->ref);
						goto error;
					}
					janus_mutex_unlock(&publisher->streams_mutex);
					/* Increase the refcount before unlocking so that nobody can remove and free the publisher in the meantime. */
					janus_refcount_increase(&publisher->ref);
					janus_refcount_increase(&publisher->session->ref);
					publishers = g_list_append(publishers, publisher);
				}
				gboolean paused = subscriber->paused;
				subscriber->paused = TRUE;
				/* Switch to the new streams, unsubscribing from the ones we replace:
				 * notice that no renegotiation happens, we just switch the sources */
				int changes = 0;
				gboolean update = FALSE;
				janus_mutex_lock(&subscriber->streams_mutex);
				for(i=0; i<json_array_size(feeds); i++) {
					json_t *s = json_array_get(feeds, i);
					/* Look for the specific subscription mid to update */
					const char *sub_mid = json_string_value(json_object_get(s, "sub_mid"));
					janus_videoroom_subscriber_stream *stream = g_hash_table_lookup(subscriber->streams_bymid, sub_mid);
					if(stream == NULL) {
						JANUS_LOG(LOG_WARN, "Subscriber stream with mid '%s' not found, not switching...\n", sub_mid);
						continue;
					}
					/* Look for the publisher stream to switch to */
					json_t *feed = json_object_get(s, "feed");
					guint64 feed_id = 0;
					char feed_id_num[30], *feed_id_str = NULL;
					if(!string_ids) {
						feed_id = json_integer_value(feed);
						g_snprintf(feed_id_num, sizeof(feed_id_num), "%"SCNu64, feed_id);
						feed_id_str = feed_id_num;
					} else {
						feed_id_str = (char *)json_string_value(feed);
					}
					const char *mid = json_string_value(json_object_get(s, "mid"));
					janus_mutex_lock(&subscriber->room->mutex);
					janus_videoroom_publisher *publisher = g_hash_table_lookup(subscriber->room->participants,
						string_ids ? (gpointer)feed_id_str : (gpointer)&feed_id);
					janus_mutex_unlock(&subscriber->room->mutex);
					if(publisher == NULL || g_atomic_int_get(&publisher->destroyed) ||
							!g_atomic_int_get(&publisher->session->started)) {
						JANUS_LOG(LOG_WARN, "Publisher '%s' not found, not switching...\n", feed_id_str);
						continue;
					}
					janus_mutex_lock(&publisher->streams_mutex);
					janus_videoroom_publisher_stream *ps = g_hash_table_lookup(publisher->streams_bymid, mid);
					janus_mutex_unlock(&publisher->streams_mutex);
					if(ps == NULL || g_atomic_int_get(&ps->destroyed)) {
						JANUS_LOG(LOG_WARN, "Publisher '%s' doesn't have any mid '%s', not switching...\n", feed_id_str, mid);
						continue;
					}
					/* If this mapping already exists, do nothing */
					if(g_slist_find(stream->publisher_streams, ps) != NULL) {
						JANUS_LOG(LOG_WARN, "Publisher '%s'/'%s' is already feeding mid '%s', not switching...\n",
							feed_id_str, mid, sub_mid);
						continue;
					}
					/* If the streams are not of the same type, do nothing */
					if(stream->type != ps->type) {
						JANUS_LOG(LOG_WARN, "Publisher '%s'/'%s' is not the same type as subscription mid '%s', not switching...\n",
							feed_id_str, mid, sub_mid);
						continue;
					}
					/* If the streams are not using the same codec, do nothing */
					if((stream->type == JANUS_VIDEOROOM_MEDIA_AUDIO && stream->acodec != ps->acodec) ||
							(stream->type == JANUS_VIDEOROOM_MEDIA_VIDEO && stream->vcodec != ps->vcodec)) {
						JANUS_LOG(LOG_WARN, "Publisher '%s'/'%s' is not using same codec as subscription mid '%s', not switching...\n",
							feed_id_str, mid, sub_mid);
						continue;
					}
					/* Unsubscribe the old stream and update it: we don't replace streams like we
					 * do when doing new subscriptions, as that might change payload type, etc. */
					changes++;
					/* Unsubscribe from the previous source first */
					janus_refcount_increase(&stream->ref);
					gboolean unref = FALSE;
					if(stream->publisher_streams == NULL) {
						/* This stream was inactive, we'll need a renegotiation */
						update = TRUE;
					} else {
						unref = TRUE;
						janus_videoroom_publisher_stream *stream_ps = stream->publisher_streams->data;
						janus_mutex_lock(&stream_ps->subscribers_mutex);
						stream_ps->subscribers = g_slist_remove(stream_ps->subscribers, stream);
						stream->publisher_streams = g_slist_remove(stream->publisher_streams, stream_ps);
						janus_mutex_unlock(&stream_ps->subscribers_mutex);
						janus_refcount_decrease(&stream_ps->ref);
					}
					/* Subscribe to the new one */
					janus_mutex_lock(&ps->subscribers_mutex);
					stream->publisher_streams = g_slist_append(stream->publisher_streams, ps);
					ps->subscribers = g_slist_append(ps->subscribers, stream);
					janus_refcount_increase(&ps->ref);
					janus_refcount_increase(&stream->ref);
					/* Reset simulcast and SVC properties too */
					janus_rtp_simulcasting_context_reset(&stream->sim_context);
					janus_mutex_lock(&ps->rid_mutex);
					stream->sim_context.rid_ext_id = ps->rid_extmap_id;
					janus_mutex_unlock(&ps->rid_mutex);
					stream->sim_context.substream_target = 2;
					stream->sim_context.templayer_target = 2;
					janus_vp8_simulcast_context_reset(&stream->vp8_context);
					stream->spatial_layer = -1;
					stream->target_spatial_layer = 1;		/* FIXME Chrome sends 0 and 1 */
					stream->temporal_layer = -1;
					stream->target_temporal_layer = 2;	/* FIXME Chrome sends 0, 1 and 2 */
					janus_mutex_unlock(&ps->subscribers_mutex);
					janus_videoroom_reqpli(ps, "Subscriber switch");
					if(unref)
						janus_refcount_decrease(&stream->ref);
					janus_refcount_decrease(&stream->ref);
				}
				janus_mutex_unlock(&subscriber->streams_mutex);
				/* Decrease the references we took before */
				while(publishers) {
					janus_videoroom_publisher *publisher = (janus_videoroom_publisher *)publishers->data;
					janus_refcount_decrease(&publisher->session->ref);
					janus_refcount_decrease(&publisher->ref);
					publishers = g_list_remove(publishers, publisher);
				}
				/* Done */
				subscriber->paused = paused;
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "switched", json_string("ok"));
				json_object_set_new(event, "room", string_ids ?
					json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "changes", json_integer(changes));
				janus_mutex_lock(&subscriber->streams_mutex);
				json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, FALSE, NULL);
				json_t *media_event = NULL;
				if(notify_events && gateway->events_is_enabled())
					media_event = json_deep_copy(media);
				janus_mutex_unlock(&subscriber->streams_mutex);
				json_object_set_new(event, "streams", media);
				/* Also notify event handlers */
				if(notify_events && gateway->events_is_enabled()) {
					json_t *info = json_object();
					json_object_set_new(info, "event", json_string("switched"));
					json_object_set_new(info, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(event, "changes", json_integer(changes));
					json_object_set_new(event, "streams", media_event);
					gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
				}
				/* Check if we need a renegotiation as well */
				if(update) {
					/* We do */
					janus_mutex_lock(&subscriber->streams_mutex);
					if(!g_atomic_int_get(&subscriber->answered)) {
						/* We're still waiting for an answer to a previous offer, postpone this */
						g_atomic_int_set(&subscriber->pending_offer, 1);
						janus_mutex_unlock(&subscriber->streams_mutex);
						JANUS_LOG(LOG_VERB, "Post-poning new offer, waiting for previous answer\n");
					} else {
						json_t *revent = json_object();
						json_object_set_new(revent, "videoroom", json_string("updated"));
						json_object_set_new(revent, "room", string_ids ?
							json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
						json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, FALSE, NULL);
						json_t *media_event = NULL;
						if(notify_events && gateway->events_is_enabled())
							media_event = json_deep_copy(media);
						json_object_set_new(revent, "streams", media);
						/* Generate a new offer */
						json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
						janus_mutex_unlock(&subscriber->streams_mutex);
						/* How long will the Janus core take to push the event? */
						gint64 start = janus_get_monotonic_time();
						int res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, revent, jsep);
						JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
						json_decref(revent);
						json_decref(jsep);
						/* Also notify event handlers */
						if(notify_events && gateway->events_is_enabled()) {
							json_t *info = json_object();
							json_object_set_new(info, "event", json_string("updated"));
							json_object_set_new(info, "room", string_ids ?
								json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
							json_object_set_new(info, "room", json_integer(subscriber->room_id));
							json_object_set_new(info, "streams", media_event);
							json_object_set_new(info, "private_id", json_integer(subscriber->pvt_id));
							gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
						}
					}
				}
			} else if(!strcasecmp(request_text, "leave")) {
				guint64 room_id = subscriber ? subscriber->room_id : 0;
				char *room_id_str = subscriber ? subscriber->room_id_str : NULL;
				/* Tell the core to tear down the PeerConnection, hangup_media will do the rest */
				janus_videoroom_hangup_media(session->handle);
				gateway->close_pc(session->handle);
				/* Send an event back */
				event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(room_id_str) : json_integer(room_id));
				json_object_set_new(event, "left", json_string("ok"));
				g_atomic_int_set(&session->started, 0);
			} else {
				JANUS_LOG(LOG_ERR, "Unknown request '%s'\n", request_text);
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_REQUEST;
				g_snprintf(error_cause, 512, "Unknown request '%s'", request_text);
				janus_refcount_decrease(&subscriber->ref);
				goto error;
			}
			janus_refcount_decrease(&subscriber->ref);
		}

		/* Prepare JSON event */
		JANUS_LOG(LOG_VERB, "Preparing JSON event as a reply\n");
		/* Any SDP or update to handle? */
		const char *msg_sdp_type = json_string_value(json_object_get(msg->jsep, "type"));
		const char *msg_sdp = json_string_value(json_object_get(msg->jsep, "sdp"));
		json_t *msg_simulcast = json_object_get(msg->jsep, "simulcast");
		gboolean e2ee = json_is_true(json_object_get(msg->jsep, "e2ee"));
		if(!msg_sdp) {
			/* No SDP to send */
			if (add_other_publishers_to_this != NULL)
				JANUS_LOG(LOG_ERR, "We wanted to add participants, but no sdp in request\n");
			int ret = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, NULL);

			JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
			json_decref(event);
		} else {
			/* Generate offer or answer */
			JANUS_LOG(LOG_VERB, "This is involving a negotiation (%s) as well:\n%s\n", msg_sdp_type, msg_sdp);
			if(sdp_update) {
				/* Renegotiation: make sure the user provided an offer, and send answer */
				JANUS_LOG(LOG_VERB, "  -- Updating existing publisher\n");
				session->sdp_version++;		/* This needs to be increased when it changes */
			} else {
				/* New PeerConnection */
				session->sdp_version = 1;	/* This needs to be increased when it changes */
				session->sdp_sessid = janus_get_real_time();
			}
			const char *type = NULL;
			if(!strcasecmp(msg_sdp_type, "offer")) {
				/* We need to answer */
				type = "answer";
			} else if(!strcasecmp(msg_sdp_type, "answer")) {
				/* We got an answer (from a subscriber?), no need to negotiate */
				g_atomic_int_set(&session->hangingup, 0);

				if (add_other_publishers_to_this != NULL)
					JANUS_LOG(LOG_ERR, "We wanted to add participants, but it was an answer\n");
				int ret = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, NULL);
				JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
				json_decref(event);
				/* Take note of the fact we got our answer */
				janus_videoroom_subscriber *subscriber = (janus_videoroom_subscriber *)session->participant;
				if(subscriber == NULL) {
					/* Shouldn't happen? */
					continue;
				}
				janus_mutex_lock(&subscriber->streams_mutex);
				/* Mark all streams that were answered to as ready */
				char error_str[512];
				janus_sdp *answer = janus_sdp_parse(msg_sdp, error_str, sizeof(error_str));
				GList *temp = answer->m_lines;
				while(temp) {
					janus_sdp_mline *m = (janus_sdp_mline *)temp->data;
					if(m->direction != JANUS_SDP_INACTIVE) {
						janus_videoroom_subscriber_stream *stream = g_hash_table_lookup(subscriber->streams_byid, GINT_TO_POINTER(m->index));
						if(stream)
							g_atomic_int_set(&stream->ready, 1);
					}
					temp = temp->next;
				}
				janus_sdp_destroy(answer);
				/* Check if we have other pending offers to send for this subscriber */
				if(g_atomic_int_compare_and_exchange(&subscriber->pending_offer, 1, 0)) {
					JANUS_LOG(LOG_VERB, "Pending offer, sending it now\n");
					event = json_object();
					json_object_set_new(event, "videoroom", json_string("updated"));
					json_object_set_new(event, "room", string_ids ?
						json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_t *media = janus_videoroom_subscriber_streams_summary(subscriber, FALSE, NULL);
					json_t *media_event = NULL;
					if(notify_events && gateway->events_is_enabled())
						media_event = json_deep_copy(media);
					json_object_set_new(event, "streams", media);
					/* Generate a new offer */
					json_t *jsep = janus_videoroom_subscriber_offer(subscriber);
					/* Do we need an ICE restart as well? */
					if(g_atomic_int_compare_and_exchange(&subscriber->pending_restart, 1, 0))
						json_object_set_new(jsep, "restart", json_true());
					janus_mutex_unlock(&subscriber->streams_mutex);
					/* How long will the Janus core take to push the event? */
					gint64 start = janus_get_monotonic_time();
					int res = gateway->push_event(session->handle, &janus_videoroom_plugin, NULL, event, jsep);
					JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (took %"SCNu64" us)\n", res, janus_get_monotonic_time()-start);
					json_decref(event);
					json_decref(jsep);
					/* Also notify event handlers */
					if(notify_events && gateway->events_is_enabled()) {
						json_t *info = json_object();
						json_object_set_new(info, "event", json_string("updated"));
						json_object_set_new(info, "room", string_ids ?
							json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
						json_object_set_new(info, "streams", media_event);
						json_object_set_new(info, "private_id", json_integer(subscriber->pvt_id));
						gateway->notify_event(&janus_videoroom_plugin, session->handle, info);
					}
				} else {
					g_atomic_int_set(&subscriber->answered, 1);
					janus_mutex_unlock(&subscriber->streams_mutex);
				}
				janus_videoroom_message_free(msg);
				continue;
			} else {
				/* TODO We don't support anything else right now... */
				JANUS_LOG(LOG_ERR, "Unknown SDP type '%s'\n", msg_sdp_type);
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_SDP_TYPE;
				g_snprintf(error_cause, 512, "Unknown SDP type '%s'", msg_sdp_type);
				goto error;
			}
			if(session->participant_type != janus_videoroom_p_type_publisher) {
				/* We shouldn't be here, we always offer ourselves */
				JANUS_LOG(LOG_ERR, "Only publishers send offers\n");
				error_code = JANUS_VIDEOROOM_ERROR_INVALID_SDP_TYPE;
				g_snprintf(error_cause, 512, "Only publishers send offers");
				goto error;
			} else {
				/* This is a new publisher, or an updated one */
				participant = janus_videoroom_session_get_publisher(session);
				janus_videoroom *videoroom = participant->room;
				int count = 0;
				GHashTableIter iter;
				gpointer value;
				if(!videoroom) {
					error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
					goto error;
				}
				if(g_atomic_int_get(&videoroom->destroyed)) {
					error_code = JANUS_VIDEOROOM_ERROR_NO_SUCH_ROOM;
					goto error;
				}
				janus_refcount_increase(&videoroom->ref);
				if(!sdp_update) {
					/* New publisher, is there room? */
					janus_mutex_lock(&videoroom->mutex);
					g_hash_table_iter_init(&iter, videoroom->participants);
					while (!g_atomic_int_get(&videoroom->destroyed) && g_hash_table_iter_next(&iter, NULL, &value)) {
						janus_videoroom_publisher *p = value;
						if(p != participant && g_atomic_int_get(&p->session->started) && !p->dummy)
							count++;
					}
					if(count == videoroom->max_publishers) {
						janus_mutex_unlock(&videoroom->mutex);
						janus_refcount_decrease(&videoroom->ref);
						JANUS_LOG(LOG_ERR, "Maximum number of publishers (%d) already reached\n", videoroom->max_publishers);
						error_code = JANUS_VIDEOROOM_ERROR_PUBLISHERS_FULL;
						g_snprintf(error_cause, 512, "Maximum number of publishers (%d) already reached", videoroom->max_publishers);
						goto error;
					}
					janus_mutex_unlock(&videoroom->mutex);
				}
				if(videoroom->require_e2ee && !e2ee && !participant->e2ee) {
					JANUS_LOG(LOG_ERR, "Room requires end-to-end encrypted media\n");
					error_code = JANUS_VIDEOROOM_ERROR_UNAUTHORIZED;
					g_snprintf(error_cause, 512, "Room requires end-to-end encrypted media");
					goto error;
				}
				/* Now prepare the SDP to give back */
				if(strstr(msg_sdp, "mozilla") || strstr(msg_sdp, "Mozilla")) {
					participant->firefox = TRUE;
				}
				/* Start by parsing the offer */
				char error_str[512];
				janus_sdp *offer = janus_sdp_parse(msg_sdp, error_str, sizeof(error_str));
				if(offer == NULL) {
					janus_refcount_decrease(&videoroom->ref);
					json_decref(event);
					JANUS_LOG(LOG_ERR, "Error parsing offer: %s\n", error_str);
					error_code = JANUS_VIDEOROOM_ERROR_INVALID_SDP;
					g_snprintf(error_cause, 512, "Error parsing offer: %s", error_str);
					goto error;
				}
				/* Prepare an answer, by iterating on all m-lines */
				janus_sdp *answer = janus_sdp_generate_answer(offer);
				json_t *media = json_array();
				json_t *descriptions = json_object_get(root, "descriptions");
				const char *audiocodec = NULL, *videocodec = NULL;
				char *vp9_profile = NULL, *h264_profile = NULL;
				GList *temp = offer->m_lines;
				janus_mutex_lock(&participant->streams_mutex);
				while(temp) {
					/* Which media are available? */
					janus_sdp_mline *m = (janus_sdp_mline *)temp->data;
					/* Check if we have a stream instance for this m-line */
					gboolean new_ps = FALSE;
					janus_videoroom_publisher_stream *ps = g_hash_table_lookup(participant->streams_byid, GINT_TO_POINTER(m->index));
					if(ps == NULL) {
						/* Initialize a new publisher stream */
						new_ps = TRUE;
						ps = g_malloc0(sizeof(janus_videoroom_publisher_stream));
						ps->type = JANUS_VIDEOROOM_MEDIA_NONE;
						if(m->type == JANUS_SDP_AUDIO)
							ps->type = JANUS_VIDEOROOM_MEDIA_AUDIO;
						else if(m->type == JANUS_SDP_VIDEO)
							ps->type = JANUS_VIDEOROOM_MEDIA_VIDEO;
						if(m->type == JANUS_SDP_APPLICATION)
							ps->type = JANUS_VIDEOROOM_MEDIA_DATA;
						ps->mindex = g_list_length(participant->streams);
						ps->publisher = participant;
						janus_refcount_increase(&participant->ref);	/* Add a reference to the publisher */
						/* Initialize the stream */
						ps->active = TRUE;
						ps->acodec = participant->acodec;
						ps->vcodec = participant->vcodec;
						ps->pt = -1;
						ps->min_delay = -1;
						ps->max_delay = -1;
						g_atomic_int_set(&ps->destroyed, 0);
						janus_refcount_init(&ps->ref, janus_videoroom_publisher_stream_free);
						g_atomic_int_inc(&g_publishers_streams_count);
						janus_refcount_increase(&ps->ref);	/* This is for the mid-indexed hashtable */
						janus_mutex_init(&ps->subscribers_mutex);
						janus_mutex_init(&ps->rtp_forwarders_mutex);
						janus_mutex_init(&ps->rid_mutex);
						ps->rtp_forwarders = g_hash_table_new_full(NULL, NULL, NULL, (GDestroyNotify)janus_videoroom_rtp_forwarder_destroy);
					}
					if(m->type == JANUS_SDP_AUDIO || m->type == JANUS_SDP_VIDEO) {
						/* Are the extmaps we care about there? */
						GList *ma = m->attributes;
						while(ma) {
							janus_sdp_attribute *a = (janus_sdp_attribute *)ma->data;
							if(a->name && a->value) {
								if(!strcasecmp(a->name, "mid")) {
									gboolean mid_changed = FALSE;
									/* Check if we're just discovering the mid or if it changed */
									if(ps->mid != NULL && strcasecmp(ps->mid, a->value))
										mid_changed = TRUE;
									char *old_mid = mid_changed ? ps->mid : NULL;
									if(ps->mid == NULL || mid_changed) {
										ps->mid = g_strdup(a->value);
										if(mid_changed) {
											/* Update the table here, since this is not a new stream */
											janus_refcount_increase(&ps->ref);
											g_hash_table_insert(participant->streams_bymid, g_strdup(ps->mid), ps);
											if(old_mid != NULL)
												g_hash_table_remove(participant->streams_bymid, old_mid);
											g_free(old_mid);
										}
									}
								} else if(videoroom->audiolevel_ext && m->type == JANUS_SDP_AUDIO &&
										ps->audio_level_extmap_id == 0 && strstr(a->value, JANUS_RTP_EXTMAP_AUDIO_LEVEL)) {
									ps->audio_level_extmap_id = atoi(a->value);
								} else if(videoroom->videoorient_ext && m->type == JANUS_SDP_VIDEO &&
										ps->video_orient_extmap_id == 0 && strstr(a->value, JANUS_RTP_EXTMAP_VIDEO_ORIENTATION)) {
									ps->video_orient_extmap_id = atoi(a->value);
								} else if(videoroom->playoutdelay_ext && m->type == JANUS_SDP_VIDEO &&
										ps->playout_delay_extmap_id == 0 && strstr(a->value, JANUS_RTP_EXTMAP_PLAYOUT_DELAY)) {
									ps->playout_delay_extmap_id = atoi(a->value);
								} else if(videoroom->do_opusfec && m->type == JANUS_SDP_AUDIO && !strcasecmp(a->name, "fmtp")) {
									if(strstr(a->value, "useinbandfec=1"))
										ps->opusfec = TRUE;
									if(strstr(a->value, "usedtx=1"))
										ps->opusdtx = TRUE;
									if(strstr(a->value, "stereo=1"))
										ps->opusstereo = TRUE;
								}
							}
							ma = ma->next;
						}
					}
					/* If this m-line is active, check the codecs we can use, or the ones we should */
					janus_sdp_mdirection mdir = JANUS_SDP_INACTIVE;
					if(m->direction != JANUS_SDP_INACTIVE) {
						if(m->type == JANUS_SDP_AUDIO) {
							if(ps->acodec != JANUS_AUDIOCODEC_NONE) {
								/* We already know which codec we'll use */
								if(ps->pt == -1 && janus_sdp_get_codec_pt(offer, m->index, janus_audiocodec_name(ps->acodec)) != -1) {
									ps->pt = janus_audiocodec_pt(ps->acodec);
								}
							} else {
								/* Check the codec priorities in the room configuration */
								int i=0;
								for(i=0; i<5; i++) {
									if(videoroom->acodec[i] == JANUS_AUDIOCODEC_NONE)
										continue;
									if(janus_sdp_get_codec_pt(offer, m->index, janus_audiocodec_name(videoroom->acodec[i])) != -1) {
										ps->acodec = videoroom->acodec[i];
										ps->pt = janus_audiocodec_pt(ps->acodec);
										break;
									}
								}
							}
							mdir = (ps->acodec != JANUS_AUDIOCODEC_NONE ? JANUS_SDP_RECVONLY : JANUS_SDP_INACTIVE);
						} else if(m->type == JANUS_SDP_VIDEO) {
							vp9_profile = videoroom->vp9_profile;
							h264_profile = videoroom->h264_profile;
							if(ps->vcodec != JANUS_VIDEOCODEC_NONE) {
								/* We already know which codec we'll use */
								if(ps->pt == -1 && ps->vcodec == JANUS_VIDEOCODEC_VP9 && vp9_profile) {
									/* Check if this VP9 profile is available */
									if(janus_sdp_get_codec_pt_full(offer, -1, janus_videocodec_name(ps->vcodec), vp9_profile) != -1) {
										/* It is */
										h264_profile = NULL;
										ps->pt = janus_videocodec_pt(ps->vcodec);
										g_free(ps->vp9_profile);
										ps->vp9_profile = g_strdup(vp9_profile);
									} else {
										/* It isn't, fallback to checking whether VP9 is available without the profile */
										vp9_profile = NULL;
									}
								} else if(ps->pt == -1 && ps->vcodec == JANUS_VIDEOCODEC_H264 && h264_profile) {
									/* Check if this H.264 profile is available */
									if(janus_sdp_get_codec_pt_full(offer, -1, janus_videocodec_name(ps->vcodec), h264_profile) != -1) {
										/* It is */
										vp9_profile = NULL;
										ps->pt = janus_videocodec_pt(ps->vcodec);
										g_free(ps->h264_profile);
										ps->h264_profile = g_strdup(h264_profile);
									} else {
										/* It isn't, fallback to checking whether H.264 is available without the profile */
										h264_profile = NULL;
									}
								}
								if(ps->pt == -1 && janus_sdp_get_codec_pt(offer, m->index, janus_videocodec_name(ps->vcodec)) != -1) {
									/* We'll only get the profile later, when we've generated an answer  */
									ps->pt = janus_videocodec_pt(ps->vcodec);
								}
							} else {
								/* Check the codec priorities in the room configuration */
								int i=0;
								for(i=0; i<5; i++) {
									if(videoroom->vcodec[i] == JANUS_VIDEOCODEC_NONE)
										continue;
									if(videoroom->vcodec[i] == JANUS_VIDEOCODEC_VP9 && vp9_profile) {
										/* Check if this VP9 profile is available */
										if(janus_sdp_get_codec_pt_full(offer, -1, janus_videocodec_name(videoroom->vcodec[i]), vp9_profile) != -1) {
											/* It is */
											h264_profile = NULL;
											ps->vcodec = videoroom->vcodec[i];
											ps->pt = janus_videocodec_pt(ps->vcodec);
											ps->vp9_profile = g_strdup(vp9_profile);
											break;
										}
										/* It isn't, fallback to checking whether VP9 is available without the profile */
										vp9_profile = NULL;
									} else if(videoroom->vcodec[i] == JANUS_VIDEOCODEC_H264 && h264_profile) {
										/* Check if this H.264 profile is available */
										if(janus_sdp_get_codec_pt_full(offer, -1, janus_videocodec_name(videoroom->vcodec[i]), h264_profile) != -1) {
											/* It is */
											vp9_profile = NULL;
											ps->vcodec = videoroom->vcodec[i];
											ps->pt = janus_videocodec_pt(ps->vcodec);
											ps->h264_profile = g_strdup(h264_profile);
											break;
										}
										/* It isn't, fallback to checking whether H.264 is available without the profile */
										h264_profile = NULL;
									}
									/* Check if the codec is available */
									if(janus_sdp_get_codec_pt(offer, m->index, janus_videocodec_name(videoroom->vcodec[i])) != -1) {
										/* We'll only get the profile later, when we've generated an answer  */
										ps->vcodec = videoroom->vcodec[i];
										ps->pt = janus_videocodec_pt(ps->vcodec);
										break;
									}
								}
							}
							/* Check if simulcast is in place */
							if(msg_simulcast != NULL && json_array_size(msg_simulcast) > 0 &&
									(ps->vcodec == JANUS_VIDEOCODEC_VP8 || ps->vcodec == JANUS_VIDEOCODEC_H264)) {
								size_t i = 0;
								for(i=0; i<json_array_size(msg_simulcast); i++) {
									json_t *s = json_array_get(msg_simulcast, i);
									int mindex = json_integer_value(json_object_get(s, "mindex"));
									if(mindex != ps->mindex)
										continue;
									JANUS_LOG(LOG_VERB, "Publisher stream is going to do simulcasting (#%d, %s)\n", ps->mindex, ps->mid);
									ps->simulcast = TRUE;
									janus_mutex_lock(&ps->rid_mutex);
									/* Clear existing RIDs in case this is a renegotiation */
									janus_rtp_simulcasting_cleanup(&ps->rid_extmap_id, NULL, ps->rid, NULL);
									janus_rtp_simulcasting_prepare(s,
										&ps->rid_extmap_id,
										ps->vssrc, ps->rid);
									janus_mutex_unlock(&ps->rid_mutex);
								}
							}
							mdir = (ps->vcodec != JANUS_VIDEOCODEC_NONE ? JANUS_SDP_RECVONLY : JANUS_SDP_INACTIVE);
						} else if(m->type == JANUS_SDP_APPLICATION) {
							mdir = JANUS_SDP_RECVONLY;
						}
					}
					ps->disabled = (mdir == JANUS_SDP_INACTIVE);
					/* Add a new m-line to the answer */
					if(m->type == JANUS_SDP_AUDIO) {
						char audio_fmtp[256];
						audio_fmtp[0] = '\0';
						if(ps->opusfec)
							g_snprintf(audio_fmtp, sizeof(audio_fmtp), "useinbandfec=1");
						if(ps->opusdtx) {
							if(strlen(audio_fmtp) == 0) {
								g_snprintf(audio_fmtp, sizeof(audio_fmtp), "usedtx=1");
							} else {
								janus_strlcat(audio_fmtp, ";usedtx=1", sizeof(audio_fmtp));
							}
						}
						if(ps->opusstereo) {
							if(strlen(audio_fmtp) == 0) {
								g_snprintf(audio_fmtp, sizeof(audio_fmtp), "stereo=1");
							} else {
								janus_strlcat(audio_fmtp, ";stereo=1", sizeof(audio_fmtp));
							}
						}
						janus_sdp_generate_answer_mline(offer, answer, m,
							JANUS_SDP_OA_MLINE, JANUS_SDP_AUDIO,
								JANUS_SDP_OA_DIRECTION, mdir,
								JANUS_SDP_OA_CODEC, janus_audiocodec_name(ps->acodec),
								JANUS_SDP_OA_FMTP, (strlen(audio_fmtp) ? audio_fmtp : NULL),
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_MID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_RID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_REPAIRED_RID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, videoroom->audiolevel_ext ? JANUS_RTP_EXTMAP_AUDIO_LEVEL : NULL,
							JANUS_SDP_OA_DONE);
						janus_sdp_mline *m_answer = janus_sdp_mline_find_by_index(answer, m->index);
						if(m_answer != NULL) {
							/* TODO Remove, this is just here for backwards compatibility */
							if(audiocodec == NULL)
								audiocodec = janus_audiocodec_name(ps->acodec);
						}
					} else if(m->type == JANUS_SDP_VIDEO) {
						janus_sdp_generate_answer_mline(offer, answer, m,
							JANUS_SDP_OA_MLINE, JANUS_SDP_VIDEO,
								JANUS_SDP_OA_DIRECTION, mdir,
								JANUS_SDP_OA_CODEC, janus_videocodec_name(ps->vcodec),
								JANUS_SDP_OA_VP9_PROFILE, vp9_profile,
								JANUS_SDP_OA_H264_PROFILE, h264_profile,
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_MID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_RID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_REPAIRED_RID,
								JANUS_SDP_OA_ACCEPT_EXTMAP, videoroom->videoorient_ext ? JANUS_RTP_EXTMAP_VIDEO_ORIENTATION : NULL,
								JANUS_SDP_OA_ACCEPT_EXTMAP, videoroom->playoutdelay_ext ? JANUS_RTP_EXTMAP_PLAYOUT_DELAY : NULL,
								JANUS_SDP_OA_ACCEPT_EXTMAP, videoroom->transport_wide_cc_ext ? JANUS_RTP_EXTMAP_TRANSPORT_WIDE_CC : NULL,
							JANUS_SDP_OA_DONE);
						janus_sdp_mline *m_answer = janus_sdp_mline_find_by_index(answer, m->index);
						if(m_answer != NULL) {
							/* TODO Remove, this is just here for backwards compatibility */
							if(videocodec == NULL)
								videocodec = janus_videocodec_name(ps->vcodec);
							/* Check if video profile has been set */
							if((ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile == NULL) || (ps->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile == NULL)) {
								int video_pt = janus_sdp_get_codec_pt(answer, m->index, janus_videocodec_name(ps->vcodec));
								const char *vfmtp = janus_sdp_get_fmtp(answer, m->index, video_pt);
								if(vfmtp != NULL) {
									if(ps->vcodec == JANUS_VIDEOCODEC_H264)
										ps->h264_profile = janus_sdp_get_video_profile(ps->vcodec, vfmtp);
									else if(ps->vcodec == JANUS_VIDEOCODEC_VP9)
										ps->vp9_profile = janus_sdp_get_video_profile(ps->vcodec, vfmtp);
								}
							}
							/* Also add a bandwidth SDP attribute if we're capping the bitrate in the room */
							if(videoroom->bitrate > 0 && videoroom->bitrate_cap) {
								if(participant->firefox) {
									/* Use TIAS (bps) instead of AS (kbps) for the b= attribute, as explained here:
									 * https://github.com/meetecho/janus-gateway/issues/1277#issuecomment-397677746 */
									m->b_name = g_strdup("TIAS");
									m->b_value = videoroom->bitrate;
								} else {
									m->b_name = g_strdup("AS");
									m->b_value = videoroom->bitrate/1000;
								}
							}
						}
					} else if(m->type == JANUS_SDP_APPLICATION) {
						janus_sdp_generate_answer_mline(offer, answer, m,
							JANUS_SDP_OA_MLINE, JANUS_SDP_APPLICATION,
							JANUS_SDP_OA_ACCEPT_EXTMAP, JANUS_RTP_EXTMAP_MID,
							JANUS_SDP_OA_DONE);
					}
					/* Make sure we have a mid */
					if(ps->mid == NULL) {
						char mid[5];
						g_snprintf(mid, sizeof(mid), "%d", ps->mindex);
						ps->mid = g_strdup(mid);
					}
					/* Do we have a description as well? */
					if(descriptions != NULL && json_array_size(descriptions) > 0) {
						size_t i = 0;
						for(i=0; i<json_array_size(descriptions); i++) {
							json_t *d = json_array_get(descriptions, i);
							const char *d_mid = json_string_value(json_object_get(d, "mid"));
							const char *d_desc = json_string_value(json_object_get(d, "description"));
							if(d_desc && d_mid && ps->mid && !strcasecmp(d_mid, ps->mid)) {
								ps->description = g_strdup(d_desc);
								break;
							}
						}
					}
					/* Add the stream to the list, if it's new */
					if(new_ps) {
						participant->streams = g_list_append(participant->streams, ps);
						if(ps->type == JANUS_VIDEOROOM_MEDIA_DATA)
							participant->data_mindex = ps->mindex;
						g_hash_table_insert(participant->streams_byid, GINT_TO_POINTER(ps->mindex), ps);
						g_hash_table_insert(participant->streams_bymid, g_strdup(ps->mid), ps);
					}
					temp = temp->next;
					/* Add to the info we send back to the publisher */
					json_t *info = json_object();
					json_object_set_new(info, "type", json_string(janus_videoroom_media_str(ps->type)));
					json_object_set_new(info, "mindex", json_integer(ps->mindex));
					json_object_set_new(info, "mid", json_string(ps->mid));
					if(ps->disabled) {
						json_object_set_new(info, "disabled", json_true());
					} else {
						if(ps->description)
							json_object_set_new(info, "description", json_string(ps->description));
						if(ps->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
							json_object_set_new(info, "codec", json_string(janus_audiocodec_name(ps->acodec)));
							if(ps->acodec == JANUS_AUDIOCODEC_OPUS) {
								if(ps->opusstereo)
									json_object_set_new(info, "stereo", json_true());
								if(ps->opusfec)
									json_object_set_new(info, "fec", json_true());
								if(ps->opusdtx)
									json_object_set_new(info, "dtx", json_true());
							}
						} else if(ps->type == JANUS_VIDEOROOM_MEDIA_VIDEO) {
							json_object_set_new(info, "codec", json_string(janus_videocodec_name(ps->vcodec)));
							if(ps->vcodec == JANUS_VIDEOCODEC_H264 && ps->h264_profile != NULL)
								json_object_set_new(info, "h264_profile", json_string(ps->h264_profile));
							else if(ps->vcodec == JANUS_VIDEOCODEC_VP9 && ps->vp9_profile != NULL)
								json_object_set_new(info, "vp9_profile", json_string(ps->vp9_profile));
							if(ps->simulcast)
								json_object_set_new(info, "simulcast", json_true());
							if(ps->svc)
								json_object_set_new(info, "svc", json_true());
						}
					}
					json_array_append_new(media, info);
				}
				janus_mutex_unlock(&participant->streams_mutex);
				janus_sdp_destroy(offer);
				/* Replace the session name */
				g_free(answer->s_name);
				char s_name[100];
				g_snprintf(s_name, sizeof(s_name), "VideoRoom %s", videoroom->room_id_str);
				answer->s_name = g_strdup(s_name);
				/* Generate an SDP string we can send back to the publisher */
				char *answer_sdp = janus_sdp_write(answer);
				janus_sdp_destroy(answer);
				/* For backwards compatibility, update the event with info on the codecs that we'll be handling
				 * TODO This will make no sense in the future, as different streams may use different codecs */
				if(event) {
					if(audiocodec)
						json_object_set_new(event, "audio_codec", json_string(audiocodec));
					if(videocodec)
						json_object_set_new(event, "video_codec", json_string(videocodec));
				}
				json_object_set_new(event, "streams", media);
				/* Is this room recorded, or are we recording this publisher already? */
				janus_mutex_lock(&participant->rec_mutex);
				if(videoroom->record || participant->recording_active) {
					GList *temp = participant->streams;
					while(temp) {
						janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)temp->data;
						janus_videoroom_recorder_create(ps);
						temp = temp->next;
					}
					participant->recording_active = TRUE;
				}
				janus_mutex_unlock(&participant->rec_mutex);
				/* Send the answer back to the publisher */
				JANUS_LOG(LOG_VERB, "Handling publisher: turned this into an '%s':\n%s\n", type, answer_sdp);
				json_t *jsep = json_pack("{ssss}", "type", type, "sdp", answer_sdp);
				g_free(answer_sdp);
				if(e2ee)
					participant->e2ee = TRUE;
				if(participant->e2ee) {
					JANUS_LOG(LOG_VERB, "Publisher is going to do end-to-end media encryption\n");
					json_object_set_new(jsep, "e2ee", json_true());
				}
				/* How long will the Janus core take to push the event? */
				g_atomic_int_set(&session->hangingup, 0);
				gint64 start = janus_get_monotonic_time();

				int res = JANUS_OK;
				if (add_other_publishers_to_this != NULL) {
					janus_mutex_lock(&videoroom->mutex);
					add_other_publishers_to_event(event, add_other_publishers_to_this, videoroom);
					res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, jsep);
					janus_mutex_unlock(&videoroom->mutex);
				} else
					res = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, jsep);
				gint64 duration = janus_get_monotonic_time()-start;
				JANUS_LOG((duration > 10000 ? LOG_WARN : LOG_VERB), "  >> Pushing event: %d (took %"SCNu64" us)\n", res, duration);

				/* If this is an update/renegotiation, notify participants about this */
				if(sdp_update && g_atomic_int_get(&session->started)) {
					/* Notify all other participants this publisher's media has changed */
					janus_videoroom_notify_about_publisher(participant, TRUE);
				}
				/* Done */
				if(res != JANUS_OK) {
					/* TODO Failed to negotiate? We should remove this publisher */
				} else {
					/* We'll wait for the setup_media event before actually telling subscribers */
				}
				janus_refcount_decrease(&videoroom->ref);
				json_decref(event);
				json_decref(jsep);
			}
			if(participant != NULL)
				janus_refcount_decrease(&participant->ref);
		}
		janus_videoroom_message_free(msg);

		continue;

error:
		{
			/* Prepare JSON error event */
			json_t *event = json_object();
			json_object_set_new(event, "videoroom", json_string("event"));
			json_object_set_new(event, "error_code", json_integer(error_code));
			json_object_set_new(event, "error", json_string(error_cause));
			int ret = gateway->push_event(msg->handle, &janus_videoroom_plugin, msg->transaction, event, NULL);
			JANUS_LOG(LOG_VERB, "  >> Pushing event: %d (%s)\n", ret, janus_get_api_error(ret));
			json_decref(event);
			janus_videoroom_message_free(msg);
		}
	}
	JANUS_LOG(LOG_VERB, "Leaving VideoRoom handler thread\n");
	return NULL;
}

static void janus_videoroom_relay_rtp_packet(gpointer data, gpointer user_data) {
	janus_videoroom_relay_rtp_packet_ex(data, user_data, FALSE);
}

/* Helper to quickly relay RTP packets from publishers to subscribers */
static void janus_videoroom_relay_rtp_packet_ex(gpointer data, gpointer user_data, gboolean mixed_audio_flag) {
	janus_videoroom_rtp_relay_packet *packet = (janus_videoroom_rtp_relay_packet *)user_data;
	if(!packet || !packet->data || packet->length < 1) {
		JANUS_LOG(LOG_ERR, "Invalid packet...\n");
		return;
	}
	janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)data;
	if(!stream || !g_atomic_int_get(&stream->ready) || g_atomic_int_get(&stream->destroyed) ||
			!stream->send || !stream->publisher_streams ||
			!stream->subscriber || stream->subscriber->paused || stream->subscriber->kicked ||
			!stream->subscriber->session || !stream->subscriber->session->handle ||
			!g_atomic_int_get(&stream->subscriber->session->started)) {
		if (!stream) {
			JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream == NULL \n");
		} else if (++stream->relay_fail_count < 20 || (unsigned)(now_ms() - stream->relay_fail_last_time) > 1000u ) {
			if (!g_atomic_int_get(&stream->ready)){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->ready\n", stream);
			} else if (g_atomic_int_get(&stream->destroyed)){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p stream->destroyed\n", stream);
			} else if (!stream->send){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->send\n", stream);
			} else if (!stream->publisher_streams){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->publisher_streams\n", stream);
			} else if (!stream->subscriber){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->subscriber\n", stream);
			} else if (stream->subscriber->paused){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p stream->subscriber->paused\n", stream);
			} else if (stream->subscriber->kicked){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p stream->subscriber->kicked\n", stream);
			} else if (!stream->subscriber->session){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->subscriber->session\n", stream);
			} else if (!stream->subscriber->session->handle){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->subscriber->session->handle\n", stream);
			} else if (!g_atomic_int_get(&stream->subscriber->session->started)){
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p !stream->subscriber->session->started\n", stream);
			} else {
				JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p race detected!\n", stream);
			}
			stream->relay_fail_last_time = now_ms();
		}
		return;
	}
	janus_videoroom_publisher_stream *ps = stream->publisher_streams ?
		stream->publisher_streams->data : NULL;
	if(ps != packet->source || ps == NULL) {
		JANUS_LOG(LOG_WARN, "VideoRoom relay fail: stream = %p,  ps = %p packet->source = %p\n", stream, ps, packet->source);
		return;
	}
	janus_videoroom_subscriber *subscriber = stream->subscriber;
	janus_videoroom_session *session = subscriber->session;

	unsigned now = now_ms();

	/* Make sure there hasn't been a publisher switch by checking the SSRC */
	if(packet->is_video) {
		/* Check if there's any SVC info to take into account */
		if(packet->svc) {
			/* There is: check if this is a layer that can be dropped for this viewer
			 * Note: Following core inspired by the excellent job done by Sergio Garcia Murillo here:
			 * https://github.com/medooze/media-server/blob/master/src/vp9/VP9LayerSelector.cpp */
			int plen = 0;
			char *payload = janus_rtp_payload((char *)packet->data, packet->length, &plen);
			gboolean keyframe = janus_vp9_is_keyframe((const char *)payload, plen);
			gboolean override_mark_bit = FALSE, has_marker_bit = packet->data->markerbit;
			int spatial_layer = stream->spatial_layer;
			gint64 now = janus_get_monotonic_time();
			if(packet->svc_info.spatial_layer >= 0 && packet->svc_info.spatial_layer <= 2)
				stream->last_spatial_layer[packet->svc_info.spatial_layer] = now;
			if(stream->target_spatial_layer > stream->spatial_layer) {
				JANUS_LOG(LOG_HUGE, "We need to upscale spatially: (%d < %d)\n",
					stream->spatial_layer, stream->target_spatial_layer);
				/* We need to upscale: wait for a keyframe */
				if(keyframe) {
					int new_spatial_layer = stream->target_spatial_layer;
					while(new_spatial_layer > stream->spatial_layer && new_spatial_layer > 0) {
						if(now - stream->last_spatial_layer[new_spatial_layer] >= 250000) {
							/* We haven't received packets from this layer for a while, try a lower layer */
							JANUS_LOG(LOG_HUGE, "Haven't received packets from layer %d for a while, trying %d instead...\n",
								new_spatial_layer, new_spatial_layer-1);
							new_spatial_layer--;
						} else {
							break;
						}
					}
					if(new_spatial_layer > stream->spatial_layer) {
						JANUS_LOG(LOG_HUGE, "  -- Upscaling spatial layer: %d --> %d (need %d)\n",
							stream->spatial_layer, new_spatial_layer, stream->target_spatial_layer);
						stream->spatial_layer = new_spatial_layer;
						spatial_layer = stream->spatial_layer;
						/* Notify the viewer */
						json_t *event = json_object();
						json_object_set_new(event, "videoroom", json_string("event"));
						json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
						json_object_set_new(event, "mid", json_string(stream->mid));
						json_object_set_new(event, "spatial_layer", json_integer(stream->spatial_layer));
						if(stream->temporal_layer == -1) {
							/* We just started: initialize the temporal layer and notify that too */
							stream->temporal_layer = 0;
							json_object_set_new(event, "temporal_layer", json_integer(stream->temporal_layer));
						}
						gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
						json_decref(event);
					}
				}
			} else if(stream->target_spatial_layer < stream->spatial_layer) {
				/* We need to downscale */
				JANUS_LOG(LOG_HUGE, "We need to downscale spatially: (%d > %d)\n",
					stream->spatial_layer, stream->target_spatial_layer);
				gboolean downscaled = FALSE;
				if(!packet->svc_info.fbit && keyframe) {
					/* Non-flexible mode: wait for a keyframe */
					downscaled = TRUE;
				} else if(packet->svc_info.fbit && packet->svc_info.ebit) {
					/* Flexible mode: check the E bit */
					downscaled = TRUE;
				}
				if(downscaled) {
					JANUS_LOG(LOG_HUGE, "  -- Downscaling spatial layer: %d --> %d\n",
						stream->spatial_layer, stream->target_spatial_layer);
					stream->spatial_layer = stream->target_spatial_layer;
					/* Notify the viewer */
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("event"));
					json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(event, "mid", json_string(stream->mid));
					json_object_set_new(event, "spatial_layer", json_integer(stream->spatial_layer));
					gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
					json_decref(event);
				}
			}
			if(spatial_layer < packet->svc_info.spatial_layer) {
				/* Drop the packet: update the context to make sure sequence number is increased normally later */
				JANUS_LOG(LOG_HUGE, "Dropping packet (spatial layer %d < %d)\n", spatial_layer, packet->svc_info.spatial_layer);
				stream->context.base_seq++;
				return;
			} else if(packet->svc_info.ebit && spatial_layer == packet->svc_info.spatial_layer) {
				/* If we stop at layer 0, we need a marker bit now, as the one from layer 1 will not be received */
				override_mark_bit = TRUE;
			}
			int temporal_layer = stream->temporal_layer;
			if(stream->target_temporal_layer > stream->temporal_layer) {
				/* We need to upscale */
				JANUS_LOG(LOG_HUGE, "We need to upscale temporally: (%d < %d)\n",
					stream->temporal_layer, stream->target_temporal_layer);
				if(packet->svc_info.ubit && packet->svc_info.bbit &&
						packet->svc_info.temporal_layer > stream->temporal_layer &&
						packet->svc_info.temporal_layer <= stream->target_temporal_layer) {
					JANUS_LOG(LOG_HUGE, "  -- Upscaling temporal layer: %d --> %d (want %d)\n",
						stream->temporal_layer, packet->svc_info.temporal_layer, stream->target_temporal_layer);
					stream->temporal_layer = packet->svc_info.temporal_layer;
					temporal_layer = stream->temporal_layer;
					/* Notify the viewer */
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("event"));
					json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(event, "mid", json_string(stream->mid));
					json_object_set_new(event, "temporal_layer", json_integer(stream->temporal_layer));
					gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
					json_decref(event);
				}
			} else if(stream->target_temporal_layer < stream->temporal_layer) {
				/* We need to downscale */
				JANUS_LOG(LOG_HUGE, "We need to downscale temporally: (%d > %d)\n",
					stream->temporal_layer, stream->target_temporal_layer);
				if(packet->svc_info.ebit && packet->svc_info.temporal_layer == stream->target_temporal_layer) {
					JANUS_LOG(LOG_HUGE, "  -- Downscaling temporal layer: %d --> %d\n",
						stream->temporal_layer, stream->target_temporal_layer);
					stream->temporal_layer = stream->target_temporal_layer;
					/* Notify the viewer */
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("event"));
					json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
					json_object_set_new(event, "mid", json_string(stream->mid));
					json_object_set_new(event, "temporal_layer", json_integer(stream->temporal_layer));
					gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
					json_decref(event);
				}
			}
			if(temporal_layer < packet->svc_info.temporal_layer) {
				/* Drop the packet: update the context to make sure sequence number is increased normally later */
				JANUS_LOG(LOG_HUGE, "Dropping packet (temporal layer %d < %d)\n", temporal_layer, packet->svc_info.temporal_layer);
				stream->context.base_seq++;
				return;
			}
			/* If we got here, we can send the frame: this doesn't necessarily mean it's
			 * one of the layers the user wants, as there may be dependencies involved */
			JANUS_LOG(LOG_HUGE, "Sending packet (spatial=%d, temporal=%d)\n",
				packet->svc_info.spatial_layer, packet->svc_info.temporal_layer);
			/* Fix sequence number and timestamp (publisher switching may be involved) */
			janus_rtp_header_update(packet->data, &stream->context, TRUE, 0);
			if(override_mark_bit && !has_marker_bit) {
				packet->data->markerbit = 1;
			}
			if(gateway != NULL) {
				janus_plugin_rtp rtp = { .mindex = stream->mindex, .video = packet->is_video, .buffer = (char *)packet->data, .length = packet->length,
					.extensions = packet->extensions };
				if(stream->min_delay > -1 && stream->max_delay > -1) {
					rtp.extensions.min_delay = stream->min_delay;
					rtp.extensions.max_delay = stream->max_delay;
				}
				janus_videoroom_stat_on_packet(stream->stat + packet->is_video, packet->length, now);
				gateway->relay_rtp(session->handle, &rtp);
			}
			if(override_mark_bit && !has_marker_bit) {
				packet->data->markerbit = 0;
			}
			/* Restore the timestamp and sequence number to what the publisher set them to */
			packet->data->timestamp = htonl(packet->timestamp);
			packet->data->seq_number = htons(packet->seq_number);
		} else if(packet->simulcast) {
			/* Handle simulcast: make sure we have a payload to work with */
			int plen = 0;
			char *payload = janus_rtp_payload((char *)packet->data, packet->length, &plen);
			if(payload == NULL)
				return;
			/* Process this packet: don't relay if it's not the SSRC/layer we wanted to handle */
			gboolean relay = janus_rtp_simulcasting_context_process_rtp(&stream->sim_context,
				(char *)packet->data, packet->length, packet->ssrc, NULL, ps->vcodec, &stream->context, &ps->rid_mutex);
			if(!relay) {
				/* Did a lot of time pass before we could relay a packet? */
				gint64 now = janus_get_monotonic_time();
				if((now - stream->sim_context.last_relayed) >= G_USEC_PER_SEC) {
					g_atomic_int_set(&stream->sim_context.need_pli, 1);
				}
			}
			if(stream->sim_context.need_pli) {
				/* Send a PLI */
				JANUS_LOG(LOG_VERB, "We need a PLI for the simulcast context\n");
				janus_videoroom_reqpli(ps, "Simulcast change");
			}
			/* Do we need to drop this? */
			if(!relay)
				return;
			/* Any event we should notify? */
			if(stream->sim_context.changed_substream) {
				/* Notify the user about the substream change */
				json_t *event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "mid", json_string(stream->mid));
				json_object_set_new(event, "substream", json_integer(stream->sim_context.substream));
				gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
				json_decref(event);
			}
			if(stream->sim_context.changed_temporal) {
				/* Notify the user about the temporal layer change */
				json_t *event = json_object();
				json_object_set_new(event, "videoroom", json_string("event"));
				json_object_set_new(event, "room", string_ids ? json_string(subscriber->room_id_str) : json_integer(subscriber->room_id));
				json_object_set_new(event, "mid", json_string(stream->mid));
				json_object_set_new(event, "temporal", json_integer(stream->sim_context.templayer));
				gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, event, NULL);
				json_decref(event);
			}
			/* If we got here, update the RTP header and send the packet */
			janus_rtp_header_update(packet->data, &stream->context, TRUE, 0);
			char vp8pd[6];
			if(ps->vcodec == JANUS_VIDEOCODEC_VP8) {
				/* For VP8, we save the original payload descriptor, to restore it after */
				memcpy(vp8pd, payload, sizeof(vp8pd));
				janus_vp8_simulcast_descriptor_update(payload, plen, &stream->vp8_context,
					stream->sim_context.changed_substream);
			}
			/* Send the packet */
			if(gateway != NULL) {
				janus_plugin_rtp rtp = { .mindex = stream->mindex, .video = packet->is_video, .buffer = (char *)packet->data, .length = packet->length,
					.extensions = packet->extensions };
				if(stream->min_delay > -1 && stream->max_delay > -1) {
					rtp.extensions.min_delay = stream->min_delay;
					rtp.extensions.max_delay = stream->max_delay;
				}
				janus_videoroom_stat_on_packet(stream->stat + packet->is_video, packet->length, now);
				gateway->relay_rtp(session->handle, &rtp);
			}
			/* Restore the timestamp and sequence number to what the publisher set them to */
			packet->data->timestamp = htonl(packet->timestamp);
			packet->data->seq_number = htons(packet->seq_number);
			if(ps->vcodec == JANUS_VIDEOCODEC_VP8) {
				/* Restore the original payload descriptor as well, as it will be needed by the next viewer */
				memcpy(payload, vp8pd, sizeof(vp8pd));
			}
		} else {
			/* Fix sequence number and timestamp (publisher switching may be involved) */
			janus_rtp_header_update(packet->data, &stream->context, TRUE, 0);
			/* Send the packet */
			if(gateway != NULL) {
				janus_plugin_rtp rtp = { .mindex = stream->mindex, .video = packet->is_video, .buffer = (char *)packet->data, .length = packet->length,
					.extensions = packet->extensions };
				if(stream->min_delay > -1 && stream->max_delay > -1) {
					rtp.extensions.min_delay = stream->min_delay;
					rtp.extensions.max_delay = stream->max_delay;
				}
				janus_videoroom_stat_on_packet(stream->stat + packet->is_video, packet->length, now);
				gateway->relay_rtp(session->handle, &rtp);
			}
			/* Restore the timestamp and sequence number to what the publisher set them to */
			packet->data->timestamp = htonl(packet->timestamp);
			packet->data->seq_number = htons(packet->seq_number);
		}
	} else {

		if(subscriber->want_mixed_audio != mixed_audio_flag) {
			return;
		}

		/* Fix sequence number and timestamp (publisher switching may be involved) */
		janus_rtp_header_update(packet->data, &stream->context, FALSE, 0);
		/* Send the packet */
		if(gateway != NULL) {
			janus_plugin_rtp rtp = { .mindex = stream->mindex, .video = packet->is_video, .buffer = (char *)packet->data, .length = packet->length,
				.extensions = packet->extensions };
			janus_videoroom_stat_on_packet(stream->stat, packet->length, now);
			gateway->relay_rtp(session->handle, &rtp);
		}
		/* Restore the timestamp and sequence number to what the publisher set them to */
		packet->data->timestamp = htonl(packet->timestamp);
		packet->data->seq_number = htons(packet->seq_number);
	}

	return;
}

static void janus_videoroom_relay_data_packet(gpointer data, gpointer user_data) {
	janus_videoroom_rtp_relay_packet *packet = (janus_videoroom_rtp_relay_packet *)user_data;
	if(!packet || packet->is_rtp || !packet->data || packet->length < 1) {
		JANUS_LOG(LOG_ERR, "Invalid packet...\n");
		return;
	}
	janus_videoroom_subscriber_stream *stream = (janus_videoroom_subscriber_stream *)data;
	if(!stream || !g_atomic_int_get(&stream->ready) || g_atomic_int_get(&stream->destroyed) ||
			!stream->send || !stream->publisher_streams ||
			!stream->subscriber || stream->subscriber->paused || stream->subscriber->kicked ||
			!stream->subscriber->session || !stream->subscriber->session->handle ||
			!g_atomic_int_get(&stream->subscriber->session->started) ||
			!g_atomic_int_get(&stream->subscriber->session->dataready))
		return;
	janus_videoroom_publisher_stream *ps = packet->source;
	if(ps->publisher == NULL || g_slist_find(stream->publisher_streams, ps) == NULL)
		return;
	janus_videoroom_subscriber *subscriber = stream->subscriber;
	janus_videoroom_session *session = subscriber->session;

	if(gateway != NULL && packet->data != NULL) {
		JANUS_LOG(LOG_VERB, "Forwarding %s DataChannel message (%d bytes) to viewer\n",
			packet->textdata ? "text" : "binary", packet->length);
		janus_plugin_data data = {
			.label = ps->publisher->user_id_str,
			.protocol = NULL,
			.binary = !packet->textdata,
			.buffer = (char *)packet->data,
			.length = packet->length
		};
		gateway->relay_data(session->handle, &data);
	}
	return;
}

/* The following methods are only relevant if RTCP is used for RTP forwarders */
static void janus_videoroom_rtp_forwarder_rtcp_receive(janus_videoroom_rtp_forwarder *forward) {
	char buffer[1500];
	struct sockaddr_storage remote_addr;
	socklen_t addrlen = sizeof(remote_addr);
	int len = recvfrom(forward->rtcp_fd, buffer, sizeof(buffer), 0, (struct sockaddr *)&remote_addr, &addrlen);
	if(len > 0 && janus_is_rtcp(buffer, len)) {
		JANUS_LOG(LOG_HUGE, "Got %s RTCP packet: %d bytes\n", forward->is_video ? "video" : "audio", len);
		/* We only handle incoming video PLIs or FIR at the moment */
		if(!janus_rtcp_has_fir(buffer, len) && !janus_rtcp_has_pli(buffer, len))
			return;
		janus_videoroom_reqpli((janus_videoroom_publisher_stream *)forward->source, "RTCP from forwarder");
	}
}

static void *janus_videoroom_rtp_forwarder_rtcp_thread(void *data) {
	JANUS_LOG(LOG_VERB, "Joining RTCP thread for RTP forwarders...\n");
	/* Run the main loop */
	g_main_loop_run(rtcpfwd_loop);
	/* When the loop ends, we're done */
	JANUS_LOG(LOG_VERB, "Leaving RTCP thread for RTP forwarders...\n");
	return NULL;
}

static unsigned now_ms(void)
{
	struct timespec now;
	clock_gettime(CLOCK_MONOTONIC, &now);
	return now.tv_sec*1000LL + now.tv_nsec/1000000;
}

static int is_screen_publisher(const char * name)
{
	if (name) {
		static const char * magic = "_screen";
		size_t len = strlen(name);
		size_t magic_len = strlen(magic);
		if (len >= magic_len && !strcmp(name + len - magic_len, magic)) {
			return 1;
		}
	}
	return 0;
}

static void speech_mix_incoming_rtp(janus_videoroom * room, speech_mix_participant_t *participant, char *buf, int len, const char * debug_tag)
{
	participant->audio_ssrc = ntohl(((janus_rtp_header *)buf)->ssrc);
	speech_mixer_put_rtp_packet(&participant->speech_mixer, (unsigned char*)buf, len, now_ms());
}

static void speech_mix_participant_free(speech_mix_participant_t * participant)
{
	char * stat = speech_mix_events_stat_string(&participant->speech_mixer);
	JANUS_LOG(LOG_INFO, "%s\n", stat);
	free(stat);
	speech_mixer_close(&participant->speech_mixer);
}

static int speech_mix_participant_init(const janus_videoroom * videoroom, speech_mix_participant_t * participant, const char * debug_tag)
{
	printf("speech_mix_participant_init %s\n", debug_tag);
	memset(participant, 0, sizeof(speech_mix_participant_t));
	speech_mixer_init(&participant->speech_mixer, debug_tag);
	return 1;
}

static void janus_videoroom_notify_multitrack_subscribers_list(janus_videoroom_subscriber *subscriber, GList *msg_list) {
	if (!subscriber || !subscriber->session || !msg_list) {
		return;
	}
	if (!subscriber->want_mixed_audio){
		return;
	}
	for (; msg_list; msg_list = msg_list->next) {
		json_t *msg = (json_t*)msg_list->data;
		JANUS_LOG(LOG_VERB, "Notifying multitrack subscriber\n");
		int ret = gateway->push_event(subscriber->session->handle, &janus_videoroom_plugin, NULL, msg, NULL);
		JANUS_LOG(LOG_VERB, "  >> %d (%s)\n", ret, janus_get_api_error(ret));
	}
}


static void speech_mix_send_to_listeners(janus_videoroom_publisher *p, int serial_id, opus_enc_t * pcommon_enc, gint16 seq, gint32 ts, GList * talking_list)
{
	GHashTableIter iter;
	gpointer key, value;
	janus_videoroom_publisher_stream *publisher_stream = NULL;
	g_hash_table_iter_init(&iter, p->streams_byid);
	// find audio stream of this publisher
	while (g_hash_table_iter_next(&iter, &key, &value)) {
		janus_videoroom_publisher_stream *ps_iter = (janus_videoroom_publisher_stream *)value;
		if (ps_iter->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
			publisher_stream = ps_iter;
			break;
		}
	}
	if (!publisher_stream) {
		return;
	}

	janus_mutex_lock(&publisher_stream->subscribers_mutex);
	GSList *slist;
	for (slist = publisher_stream->subscribers; slist; slist = slist->next) {
		janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)slist->data;
		if (ss->subscriber->want_mixed_audio && ss->subscriber->serial_id != serial_id) {
			janus_videoroom_rtp_relay_packet outpkt[1];

			ss->subscriber->mixer_pvt_id = p->pvt_id;
			JANUS_LOG(LOG_WARN, "using participant %s for sending mixed audio to listen-only participant with pvt_it = %u\n", p->display, ss->subscriber->pvt_id);

			// send
			ss->subscriber->serial_id = serial_id;
			static unsigned char payload[1500] = {0,};
			memcpy(payload + 12, pcommon_enc->bitstream, pcommon_enc->bitstream_bytes);
			outpkt->data = (janus_rtp_header*)payload;
			outpkt->length = pcommon_enc->bitstream_bytes;
			outpkt->length += 12;	/* Take the RTP header into consideration */
			outpkt->data->version = 2;
			outpkt->data->markerbit = 0;	/* FIXME Should be 1 for the first packet */
			GSList *slist;
			gboolean success = FALSE;
			for (slist = ss->publisher_streams; slist; slist = slist->next) {
				janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)slist->data;
				if (ps->type==JANUS_VIDEOROOM_MEDIA_AUDIO) {
					outpkt->source = ps;
					outpkt->data->ssrc = htonl(ps->publisher->speech_mix_participant.audio_ssrc);
					outpkt->ssrc[0] = ps->publisher->speech_mix_participant.audio_ssrc;
					outpkt->data->type = ps->pt;
					outpkt->data->seq_number = htons(seq);
					outpkt->data->timestamp = htonl(ts);
					outpkt->data->type = janus_audiocodec_pt(publisher_stream->acodec);
					outpkt->is_video = FALSE;
					janus_videoroom_relay_rtp_packet(ss, outpkt);
					janus_videoroom_notify_multitrack_subscribers_list(ss->subscriber, talking_list);
					success = TRUE;
					break;
				}
			}
			if (!success) {
				JANUS_LOG(LOG_WARN, "ERROR using participant %s for sending mixed audio to listen-only participant with pvt_it = %u: participant have no audio track!\n", p->display, ss->subscriber->pvt_id);
			}
		}
	}
	janus_mutex_unlock(&publisher_stream->subscribers_mutex);
}

static void *speech_mix_thread(void *data)
{
	janus_videoroom *room = (janus_videoroom *)data;
	opus_enc_t common_enc;
	opus_enc_init(&common_enc, "common_enc");

	if (!room) {
		JANUS_LOG(LOG_ERR, "Invalid room!\n");
		return NULL;
	}
	int serial_id = 1;
	JANUS_LOG(LOG_VERB, "Thread is for mixing room %"SCNu64" (%s)\n", room->room_id, room->room_name);

	/* Buffer (we allocate assuming 48kHz, although we'll likely use less than that) */
	int frame_size_ms = 20;
	int samples = MIX_SAMPLE_RATE * frame_size_ms / 1000;

	/* Timer */
	struct timeval now, before;
	gettimeofday(&before, NULL);
	now.tv_sec = before.tv_sec;
	now.tv_usec = before.tv_usec;

	time_t passed, d_s, d_us;

	/* RTP */
	gint16 seq = 0;
	gint32 ts = 0;

	/* Loop */
	unsigned int count = 0, prev_count = 0;
	while (!g_atomic_int_get(&stopping) && !g_atomic_int_get(&room->destroyed)) {
		janus_videoroom_rtp_relay_packet outpkt[1];
		/* See if it's time to prepare a frame */
		gettimeofday(&now, NULL);
		d_s = now.tv_sec - before.tv_sec;
		d_us = now.tv_usec - before.tv_usec;
		if (d_us < 0) {
			d_us += 1000000;
			--d_s;
		}

		passed = d_s * 1000000 + d_us;

		if (passed > 200 * 1000) {
			if (passed > 500 * 1000) {
				JANUS_LOG(LOG_WARN, "Long sleep (%d ms) in room %"SCNu64" (%s)...\n", (int)(passed / 1000), room->room_id, room->room_name);
			} else {
				JANUS_LOG(LOG_INFO, "Long sleep (%d ms) in room %"SCNu64" (%s)...\n", (int)(passed / 1000), room->room_id, room->room_name);
			}
		}


		if (passed < 15000) {	/* Let's wait about 15ms at max */
			g_usleep(5000);
			continue;
		}

		/* Do we need to mix at all? */
		janus_mutex_lock_nodebug(&room->mutex);
		count = g_hash_table_size(room->participants);
		if (count < room->min_audio_mix_participants) {
			/* No participant and RTP forwarders, do nothing */
			if (prev_count >= room->min_audio_mix_participants) {
				JANUS_LOG(LOG_VERB, "Last user/forwarder just left room %"SCNu64", going idle...\n", room->room_id);
				prev_count = 0;
			}
			janus_mutex_unlock_nodebug(&room->mutex);
			/* Update the reference time */
			before.tv_usec += frame_size_ms * 1000;
			if (before.tv_usec > 1000000) {
				before.tv_sec++;
				before.tv_usec -= 1000000;
			}
			continue;
		}
		if (prev_count == 0) {
			JANUS_LOG(LOG_VERB, "First user/forwarder just joined room %"SCNu64", waking it up...\n", room->room_id);
		}
		prev_count = count;

		/* Mix all contributions */
		GList *ps, *participants_list = NULL;
		GHashTableIter iter;
		gpointer key, value;
		g_hash_table_iter_init(&iter, room->participants);
		while (g_hash_table_iter_next(&iter, &key, &value)) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)value;
			if (g_atomic_int_get(&p->destroyed) || p->kicked || p->room == NULL || p->session == NULL || g_atomic_int_get(&p->session->destroyed))
				continue;
			p = janus_videoroom_session_get_publisher_nodebug(p->session);
			if (!p)
				continue;
			if(is_screen_publisher(p->display)) {
				janus_videoroom_publisher_dereference_nodebug(p);
				continue;
			}
			if(!strcmp(g_audio_mix_dummy_publisher_name, p->display)) {
				janus_videoroom_publisher_dereference_nodebug(p);
				continue;
			}

			janus_refcount_increase(&p->session->ref);
			participants_list = g_list_prepend(participants_list, p);
		}

		speech_mixer_t * mix_list = NULL;
		for (ps = participants_list; ps; ps = ps->next) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			p->speech_mix_participant.speech_mixer.next = mix_list;
			p->speech_mix_participant.speech_mixer.user_data = p;
			mix_list = &p->speech_mix_participant.speech_mixer;
		}

		/* Update RTP header information */
		seq++;
		ts += frame_size_ms * MIX_SAMPLE_RATE / 1000;
		/* Update the reference time */
		before.tv_usec += frame_size_ms * 1000;
		if (before.tv_usec > 1000000) {
			before.tv_sec++;
			before.tv_usec -= 1000000;
		}

		speech_mix_run(mix_list, &common_enc, samples, now_ms());

		// report all talkers
		GList * talking_list = NULL;
		for (ps = participants_list; ps; ps = ps->next) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			if (p->speech_mix_participant.speech_mixer.activity_countdown <= 0 ||
			    p->speech_mix_participant.speech_mixer.mean_activity < 0.01) {
				// not mixed, or too small level
				if (p->last_audio_level_reported != 0) {
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("mixer-stopped-talking"));
					json_object_set_new(event, "room", string_ids ? json_string(room->room_id_str) : json_integer(room->room_id));
					json_object_set_new(event, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
					talking_list = g_list_prepend(talking_list, event);
					p->last_audio_level_reported = 0;
					p->last_audio_level_report_time = now_ms();
					if (string_ids)
						JANUS_LOG(LOG_INFO, "Not talking %s %s in room %"SCNu64" (%s)...\n", p->user_id_str, p->display, room->room_id, room->room_name);
					else
						JANUS_LOG(LOG_INFO, "Not talking %"SCNu64" in room %"SCNu64" (%s)...\n", p->user_id, room->room_id, room->room_name);
				}
			} else {
				int db = (int)-10*log10(p->speech_mix_participant.speech_mixer.mean_activity/32768);
				db = MIN(MAX(db, 1), 127);
				if ((p->last_audio_level_reported == 0 || labs(db - p->last_audio_level_reported) > 10)
				     && now_ms() - p->last_audio_level_report_time > AUDIO_LEVEL_REPORT_INTERVAL_MS) {
					json_t *event = json_object();
					json_object_set_new(event, "videoroom", json_string("mixer-talking"));
					json_object_set_new(event, "room", string_ids ? json_string(room->room_id_str) : json_integer(room->room_id));
					json_object_set_new(event, "id", string_ids ? json_string(p->user_id_str) : json_integer(p->user_id));
					json_object_set_new(event, "audio-level-dBov-avg", json_real(db));
					talking_list = g_list_prepend(talking_list, event);
					p->last_audio_level_reported = db;
					p->last_audio_level_report_time = now_ms();
					if (string_ids)
						JANUS_LOG(LOG_INFO, "Now talking %s %s in room %"SCNu64" (%s)  %d dB...\n", p->user_id_str, p->display, room->room_id, room->room_name, db);
					else
						JANUS_LOG(LOG_INFO, "Now talking %"SCNu64" in room %"SCNu64" (%s)  %d dB...\n", p->user_id, room->room_id, room->room_name, db);
				}
			}
		}

		janus_mutex_unlock_nodebug(&room->mutex);
		serial_id++;

		/* Send proper packet to each participant (remove own contribution) */
		for (ps = participants_list; ps; ps = ps->next) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			speech_mix_participant_t * sink = &p->speech_mix_participant;

			janus_videoroom_publisher_stream *publisher_stream = NULL;
			g_hash_table_iter_init(&iter, p->streams_byid);
			while (g_hash_table_iter_next(&iter, &key, &value)) {
				janus_videoroom_publisher_stream *ps_iter = (janus_videoroom_publisher_stream *)value;
				if (ps_iter->type == JANUS_VIDEOROOM_MEDIA_AUDIO) {
					publisher_stream = ps_iter;
					break;
				}
			}
			if (!publisher_stream) {
				continue;
			}

			/* Output buffer */
			unsigned char payload[1500] = {0,};
			outpkt->data = (janus_rtp_header*)payload;
			memcpy(payload + 12, sink->speech_mixer.opus_enc.bitstream, sink->speech_mixer.opus_enc.bitstream_bytes);
			outpkt->length = sink->speech_mixer.opus_enc.bitstream_bytes;

			if (outpkt->length < 0) {
				JANUS_LOG(LOG_ERR, "[Opus] Ops! got an error encoding the Opus frame: %d (%s)\n", outpkt->length, opus_strerror(outpkt->length));
			} else {
				outpkt->length += 12;	/* Take the RTP header into consideration */
				janus_mutex_lock_nodebug(&p->subscribers_mutex);
				if (p->subscriptions) {
					// mark all subscriptions of this publisher as 'updated' with new serial packet id
					janus_videoroom_subscriber *subscriber = NULL;
					janus_videoroom_subscriber_stream *subscriber_ss;
					janus_videoroom_publisher_stream *subscriber_ps;
					gboolean publisher_found = FALSE;

					GSList *slist, *pslist;
					GList *sslist;
					for (slist = p->subscriptions; slist && !publisher_found; slist = slist->next) {
						janus_videoroom_subscriber *subscriber_it = (janus_videoroom_subscriber *)slist->data;
						for (sslist = subscriber_it->streams; sslist && !publisher_found; sslist = sslist->next) {
							janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)sslist->data;
							for (pslist = ss->publisher_streams; pslist; pslist = pslist->next) {
								janus_videoroom_publisher_stream *ps = (janus_videoroom_publisher_stream *)pslist->data;
								if (ps->type==JANUS_VIDEOROOM_MEDIA_AUDIO && ps->publisher && !is_screen_publisher(ps->publisher->display)) {
									subscriber = subscriber_it;
									subscriber_ss = ss;
									subscriber_ps = ps;
									subscriber_it->serial_id = serial_id;
									if (!strcmp(g_audio_mix_dummy_publisher_name, ps->publisher->display)) {
										publisher_found = TRUE;
										break;
									}
								}
							}
						}
					}
					if (!subscriber) {
						// take a 1st subscriptions of this publisher
						JANUS_LOG(LOG_WARN, "no valid subscribers found for %s\n", p->display);
					} else if (subscriber->want_mixed_audio) {
						if (sink->subscriber_pvt_id != subscriber_ps->publisher->pvt_id) {
							sink->subscriber_pvt_id = subscriber_ps->publisher->pvt_id;
							JANUS_LOG(LOG_WARN, "using %s (pvt_it = %u) for sending mixed audio to %s pvt_id = %u id=%s...\n", subscriber_ps->publisher->display, subscriber_ps->publisher->pvt_id, p->display, p->pvt_id, p->user_id_str);
						}


						if (subscriber->session && subscriber->session->sdp_sessid != sink->sdp_sessid) {
							sink->sdp_sessid = subscriber->session->sdp_sessid;
							JANUS_LOG(LOG_WARN, "using sessid %"SCNu64" for sending mixed audio to %s...\n", sink->sdp_sessid, p->user_id_str);
						}

						outpkt->source = publisher_stream;
						outpkt->data->version = 2;
						outpkt->data->markerbit = 0;	/* FIXME Should be 1 for the first packet */
						outpkt->data->seq_number = htons(seq);
						outpkt->data->timestamp = htonl(ts);
						outpkt->data->type = publisher_stream->pt;
						outpkt->is_video = FALSE;
						//outpkt->data->ssrc = htonl(publisher_stream->publisher->speech_mix_participant.audio_ssrc);
						outpkt->data->ssrc = htonl(p->speech_mix_participant.audio_ssrc);
						outpkt->ssrc[0] = outpkt->data->ssrc;//htonl(outpkt->data->ssrc); //??

						// audio level notification
						outpkt->source = subscriber_ps;
						janus_videoroom_relay_rtp_packet_ex(subscriber_ss, outpkt, TRUE);
						janus_videoroom_notify_multitrack_subscribers_list(subscriber, talking_list);
					}
				}
				janus_mutex_unlock_nodebug(&p->subscribers_mutex);
			}
		}

		// this loop is for subscribers, which are not publishers:
		// for each publishing stream find subscriber with old serial id
		// send audio to all subscribers of "audiomix" dummy publisher
		for (ps = participants_list; ps; ps = ps->next) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			if (!strcmp(g_audio_mix_dummy_publisher_name, p->display)) {
				speech_mix_send_to_listeners(p, serial_id, &common_enc, seq, ts, talking_list);
			}
		}

		// send audio to all subscribers of all publishers (if not send above)
		for (ps = participants_list; ps; ps = ps->next) {
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			speech_mix_send_to_listeners(p, serial_id, &common_enc, seq, ts, talking_list);
		}

		for (ps = talking_list; ps; ps = ps->next) {
			json_t *msg = (json_t*)ps->data;
			json_decref(msg);
		}
		g_list_free(talking_list);

		for (ps = participants_list; ps; ps = ps->next)
		{
			janus_videoroom_publisher *p = (janus_videoroom_publisher *)ps->data;
			janus_refcount_decrease(&p->session->ref);
			janus_videoroom_publisher_dereference_nodebug(p);
		}
		g_list_free(participants_list);
		print_resource_usage(NULL);
	}
	JANUS_LOG(LOG_VERB, "Leaving mixer thread for room %"SCNu64" (%s)...\n", room->room_id, room->room_name);

	janus_refcount_decrease(&room->ref);
	opus_enc_close(&common_enc);
	return NULL;
}

void janus_videoroom_outgoing_congestion(janus_plugin_session *handle, int congestion_flag)
{
	JANUS_LOG(LOG_INFO, "Simulcasting try set congestion_flag = %d...\n", congestion_flag);
	if(handle == NULL || g_atomic_int_get(&handle->stopped) || g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return;
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session || g_atomic_int_get(&session->destroyed) || session->participant_type != janus_videoroom_p_type_subscriber)
		return;
	janus_videoroom_subscriber *subscriber = janus_videoroom_session_get_subscriber_nodebug(session);
	if(subscriber == NULL)
		return;
	if(g_atomic_int_get(&subscriber->destroyed)) {
		janus_refcount_decrease_nodebug(&subscriber->ref);
		return;
		}

	GList *list;
	for (list = subscriber->streams; list; list = list->next) {
		janus_videoroom_subscriber_stream *ss = (janus_videoroom_subscriber_stream *)list->data;
		ss->sim_context.transport_cc_congestion_detected = congestion_flag;
	}

	janus_refcount_decrease_nodebug(&subscriber->ref);
}

int janus_videoroom_query_user_name(janus_plugin_session *handle, char * name, int name_size)
{
	int size_copyed = 0;
	if(handle == NULL || g_atomic_int_get(&handle->stopped) || g_atomic_int_get(&stopping) || !g_atomic_int_get(&initialized))
		return 0;
	janus_videoroom_session *session = (janus_videoroom_session *)handle->plugin_handle;
	if(!session || g_atomic_int_get(&session->destroyed))
		return 0;

	if(session->participant_type == janus_videoroom_p_type_subscriber) {
		janus_videoroom_subscriber *subscriber = janus_videoroom_session_get_subscriber_nodebug(session);
		if(subscriber) {
			if(!g_atomic_int_get(&subscriber->destroyed) && subscriber->room && subscriber->room->private_ids) {
				janus_videoroom_publisher *owner =  g_hash_table_lookup(subscriber->room->private_ids, GUINT_TO_POINTER(subscriber->pvt_id));
				if(owner == NULL || g_atomic_int_get(&owner->destroyed)) {
					strncpy(name, "listen-only participant!", name_size);
				} else {
					strncpy(name, owner->display, name_size);
					janus_mutex_lock(&subscriber->streams_mutex);
					GList *temp = subscriber->streams;
					guint streams_count = g_list_length(temp);
					while(temp) {
						janus_videoroom_subscriber_stream * ss = (janus_videoroom_subscriber_stream *)temp->data;
						janus_videoroom_publisher_stream * ps = (janus_videoroom_publisher_stream *)(ss->publisher_streams?ss->publisher_streams->data:NULL);
						snprintf(name, name_size, "%s watching %d streams (%s +)", owner->display, streams_count, ps?ps->publisher->display:"NULL");
						break; // TODO: ignore other publishers, if they are in the same peer connection
						//temp = temp->next;
					}
					janus_mutex_unlock(&subscriber->streams_mutex);
				}

				size_copyed = strlen(name);
			}
			janus_refcount_decrease_nodebug(&subscriber->ref);
		}
		return size_copyed;
	} else if (session->participant_type == janus_videoroom_p_type_publisher) {
		janus_videoroom_publisher *publisher = janus_videoroom_session_get_publisher_nodebug(session);
		if (publisher) {
			if(!g_atomic_int_get(&publisher->destroyed)) {
				strncpy(name, publisher->display, name_size);
				size_copyed = strlen(name);
			}
			janus_refcount_decrease_nodebug(&publisher->ref);
		}
		return size_copyed;
	}
	return 0;
}
