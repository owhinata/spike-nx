using System.Collections.Immutable;
using ImuViewer.Core.Wire;

namespace ImuViewer.Core.LegoSensor;

/// <summary>
/// Per-class state aggregator for LEGO sensor TLV streams.  Maintains
/// six fixed buckets (one per <see cref="LegoClassId"/>) so panels in
/// the UI are stable across attach/detach.  The wire layer
/// (<see cref="Btsensor.BtsensorSession"/>) hands every BUNDLE's TLV
/// section to <see cref="OnTlvBatch"/>; subscribers consume the four
/// events below.
/// </summary>
public sealed class LegoSampleAggregator
{
    /// <summary>
    /// After this many consecutive bundles without a BOUND TLV for a
    /// class, mark the class unbound (publishes <see cref="StatusChanged"/>).
    /// </summary>
    public int UnboundAfterMissedBundles { get; init; } = 50;

    public sealed class ClassState
    {
        public LegoClassId ClassId { get; init; }
        public bool IsBound { get; internal set; }
        public byte? PortId { get; internal set; }
        public byte? ModeId { get; internal set; }
        public byte Age10ms { get; internal set; } = 0xFF;
        public ushort? LastSeq { get; internal set; }
        public LegoSamplePoint? LastSample { get; internal set; }
        internal int MissedBundles;
    }

    private readonly Dictionary<LegoClassId, ClassState> _state;

    public LegoSampleAggregator()
    {
        _state = new();
        foreach (LegoClassId id in Enum.GetValues<LegoClassId>())
        {
            _state[id] = new ClassState { ClassId = id };
        }
    }

    public IReadOnlyDictionary<LegoClassId, ClassState> State => _state;

    /// <summary>
    /// Fired for each FRESH TLV with its decoded sample.  Note the same
    /// bundle that fires this event also fires <see cref="StatusChanged"/>
    /// (covers age / mode_id updates that don't carry a payload).
    /// </summary>
    public event Action<LegoClassId, LegoSamplePoint>? SampleReceived;

    /// <summary>Fired when any of port_id/mode_id/IsBound changes.</summary>
    public event Action<LegoClassId, ClassState>? StatusChanged;

    /// <summary>
    /// Fired when port_id or mode_id changes within a single class.
    /// Subscribers should clear plot history because the value semantics
    /// have changed.  Always preceded by a <see cref="StatusChanged"/>.
    /// </summary>
    public event Action<LegoClassId, ClassState>? PortChanged;

    /// <summary>
    /// Fired once per bundle, after all per-class state has been updated.
    /// Useful for UI consumers that want a single redraw barrier.
    /// </summary>
    public event Action? BundleProcessed;

    public void OnTlvBatch(uint tickTsUs, ImmutableArray<LegoTlv> tlvs)
    {
        foreach (LegoTlv tlv in tlvs)
        {
            if (!_state.TryGetValue(tlv.ClassId, out ClassState? cls))
            {
                continue;       // unknown class, skip silently
            }

            ProcessOneTlv(cls, tickTsUs, tlv);
        }

        // Apply the missed-bundle counter for any class whose TLV said
        // BOUND=false this bundle (only counts when we never saw bound
        // in this bundle for that class).  Since the firmware emits 6
        // TLVs per bundle, every class is touched, so this loop is just
        // a guard against a future protocol change.
        foreach (ClassState cls in _state.Values)
        {
            if (cls.IsBound && cls.MissedBundles >= UnboundAfterMissedBundles)
            {
                cls.IsBound = false;
                cls.PortId = null;
                cls.ModeId = null;
                StatusChanged?.Invoke(cls.ClassId, cls);
            }
        }

        BundleProcessed?.Invoke();
    }

    private void ProcessOneTlv(ClassState cls, uint tickTsUs, LegoTlv tlv)
    {
        bool prevBound = cls.IsBound;
        byte? prevPort = cls.PortId;
        byte? prevMode = cls.ModeId;

        if (tlv.IsBound)
        {
            cls.MissedBundles = 0;
            cls.IsBound = true;
            cls.PortId = tlv.PortId;
            cls.ModeId = tlv.ModeId;
            cls.Age10ms = tlv.Age10ms;
        }
        else
        {
            cls.MissedBundles++;
            if (cls.MissedBundles >= UnboundAfterMissedBundles)
            {
                cls.IsBound = false;
                cls.PortId = null;
                cls.ModeId = null;
            }
        }

        bool boundFlipped   = prevBound != cls.IsBound;
        bool portModeChanged = prevPort != cls.PortId || prevMode != cls.ModeId;

        if (boundFlipped || portModeChanged)
        {
            StatusChanged?.Invoke(cls.ClassId, cls);
            // PortChanged fires only when transitioning from one bound
            // (port, mode) to another — first bind from unbound is a
            // status change, not a port-or-mode swap.
            if (cls.IsBound && prevBound && portModeChanged)
            {
                PortChanged?.Invoke(cls.ClassId, cls);
            }
        }

        if (tlv.IsFresh && tlv.Payload.Length > 0)
        {
            (string label, IReadOnlyList<float> values, IReadOnlyList<string> units) =
                ScaleTables.Decode(tlv.ClassId, tlv.ModeId, tlv.DataType,
                                   tlv.NumValues, tlv.Payload.AsSpan());
            cls.LastSeq = tlv.Seq;
            cls.Age10ms = 0;

            LegoSamplePoint pt = new(
                TickTsUs: tickTsUs,
                ModeId: tlv.ModeId,
                Label: label,
                Values: ImmutableArray.CreateRange(values),
                Units: ImmutableArray.CreateRange(units));
            cls.LastSample = pt;
            SampleReceived?.Invoke(cls.ClassId, pt);
        }
    }
}
