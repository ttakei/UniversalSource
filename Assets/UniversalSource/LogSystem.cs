using System.Collections.Generic;
using System.Globalization;
using UnityEngine;

namespace UniversalSource
{
    /// <summary>
    /// 4ノードのセルで、A→B→C と A→D→C の到着差を曲率のログとして残す。
    /// StabilizeStep は静止補正を、スピンが作る非可換なズレへ寄せる。ゼロには潰さない。
    /// 横軸はログ自身の歩数であり、ワールド時刻でも絶対座標でもない。
    /// </summary>
    public readonly struct CurvatureResidual
    {
        public CurvatureResidual(
            int logStep,
            int sourceProperTime,
            int arrivalProperTime,
            Vector4 residual,
            float energy,
            float plaquetteEnergy,
            float spinPlaquetteEnergy,
            Vector4 alongAbc,
            Vector4 alongAdc)
        {
            LogStep = logStep;
            SourceProperTime = sourceProperTime;
            ArrivalProperTime = arrivalProperTime;
            Residual = residual;
            Energy = energy;
            PlaquetteEnergy = plaquetteEnergy;
            SpinPlaquetteEnergy = spinPlaquetteEnergy;
            AlongAbc = alongAbc;
            AlongAdc = alongAdc;
        }

        public int LogStep { get; }
        public int SourceProperTime { get; }
        public int ArrivalProperTime { get; }
        public Vector4 Residual { get; }
        public float Energy { get; }
        public float PlaquetteEnergy { get; }
        public float SpinPlaquetteEnergy { get; }
        public Vector4 AlongAbc { get; }
        public Vector4 AlongAdc { get; }
    }

    [DisallowMultipleComponent]
    [DefaultExecutionOrder(100)]
    [AddComponentMenu("UniversalSource/Log System")]
    public sealed class LogSystem : MonoBehaviour
    {
        public const int LogCapacity = 96;
        const float SingularDeterminant = 1e-6f;
        const float ExcessRiseTolerance = 0.05f;

        [SerializeField] SourceNode nodeA = null;
        [SerializeField] SourceNode nodeB = null;
        [SerializeField] SourceNode nodeC = null;
        [SerializeField] SourceNode nodeD = null;
        [SerializeField] UniversalLink linkAB = null;
        [SerializeField] UniversalLink linkBC = null;
        [SerializeField] UniversalLink linkAD = null;
        [SerializeField] UniversalLink linkDC = null;
        [SerializeField] float feedbackGain = 0.2f;
        [SerializeField] int demoStepsOnStart = 0;
        [SerializeField] bool liveStep = true;
        [SerializeField] int framesPerStep = 6;
        [SerializeField] bool showOverlay = true;

        readonly List<CurvatureResidual> log = new List<CurvatureResidual>(LogCapacity);
        int logStep;
        int feedbackStep;
        float initialPlaquetteEnergy = -1f;
        float accumulatedLogEnergy;
        string overlayText = "";
        string status = "";
        Vector2 scroll;
        GUIStyle bodyStyle;
        int frameCursor;

        public IReadOnlyList<CurvatureResidual> Residuals => log;
        public float InitialPlaquetteEnergy => initialPlaquetteEnergy;
        public float AccumulatedLogEnergy => accumulatedLogEnergy;
        public string Status => status;

        public float CurrentPlaquetteEnergy
        {
            get { return log.Count == 0 ? 0f : log[log.Count - 1].PlaquetteEnergy; }
        }

        public CurvatureResidual Latest
        {
            get { return log.Count == 0 ? default : log[log.Count - 1]; }
        }

        void Start()
        {
            if (!RecordCurrent())
            {
                Debug.LogWarning(status);
                return;
            }

            RunSteps(demoStepsOnStart);
            LogSteadyState();
        }

        void Update()
        {
            if (!liveStep)
                return;

            frameCursor++;
            if (frameCursor < Mathf.Max(1, framesPerStep))
                return;

            frameCursor = 0;
            if (!StabilizeStep())
                return;

            if (feedbackStep <= 4 || feedbackStep % 10 == 0)
                LogSteadyState();
        }

        void OnGUI()
        {
            if (!showOverlay)
                return;

            if (bodyStyle == null)
            {
                bodyStyle = new GUIStyle(GUI.skin.label)
                {
                    fontSize = 14,
                    wordWrap = true,
                    richText = false
                };
                bodyStyle.normal.textColor = new Color(0.1f, 0.12f, 0.14f);
            }

            SampleOverlayLayout.GetRects(out _, out var area);
            GUILayout.BeginArea(area, GUI.skin.box);
            if (GUILayout.Button("安定化を8ステップ進める"))
            {
                RunSteps(8);
                LogSteadyState();
            }

            var chart = GUILayoutUtility.GetRect(10f, 150f, GUILayout.ExpandWidth(true));
            DrawEnergyChart(chart);
            scroll = GUILayout.BeginScrollView(scroll);
            GUILayout.Label(string.IsNullOrEmpty(overlayText) ? "ログ待ち" : overlayText, bodyStyle);
            GUILayout.EndScrollView();
            GUILayout.EndArea();
        }

        /// <summary>セルの残差を測る。ログへは足さない。</summary>
        public bool TryMeasureCurvature(out CurvatureResidual residual)
        {
            residual = default;
            if (!HasCell() || !TryReadPathMatrices(out var ab, out var bc, out var ad, out var dc))
            {
                status = "セルのリンクを読めない。";
                return false;
            }

            var pathAbc = bc * ab;
            var pathAdc = dc * ad;
            if (!TrySpinProducts(out var spinAbc, out var spinAdc))
            {
                status = "スピン結合を計算できない。";
                return false;
            }

            var probe = CommunicationProbe();
            if (!UniversalLink.TryTransportAlong(nodeA, probe, out var alongAbc, linkAB, linkBC)
                || !UniversalLink.TryTransportAlong(nodeA, probe, out var alongAdc, linkAD, linkDC))
            {
                status = "経路の転送を完了できない。";
                return false;
            }

            var residualVector = alongAbc - alongAdc;
            residual = new CurvatureResidual(
                logStep,
                nodeA.ProperTime,
                nodeC.ProperTime,
                residualVector,
                residualVector.sqrMagnitude,
                FrobeniusGap(pathAbc, pathAdc),
                FrobeniusGap(spinAbc, spinAdc),
                alongAbc,
                alongAdc);
            return true;
        }

        /// <summary>いまの残差をログへ1件足す。</summary>
        public bool RecordCurrent()
        {
            if (!TryMeasureCurvature(out var residual))
                return false;

            Append(residual);
            logStep++;
            RefreshOverlay();
            return true;
        }

        /// <summary>
        /// 各ノードのスピンを1歩進め、静止補正をスピン由来の経路差へ寄せてログへ残す。
        /// 偶数歩は到着側（B→C と D→C）、奇数歩は出発側（A→B と A→D）を動かす。
        /// </summary>
        [ContextMenu("安定化を1ステップ")]
        public bool StabilizeStep()
        {
            if (!HasCell())
            {
                status = "セルの参照を確認する。";
                RefreshOverlay();
                return false;
            }

            if (log.Count == 0 && !RecordCurrent())
                return false;

            AdvanceSpins();
            if (!TryReadPathMatrices(out var ab, out var bc, out var ad, out var dc)
                || !TrySpinProducts(out var spinAbc, out var spinAdc))
                return false;

            var before = FrobeniusGap(bc * ab, dc * ad);
            var spinGap = FrobeniusGap(spinAbc, spinAdc);
            var excessBefore = Mathf.Abs(before - spinGap);
            var stored = CaptureStored();
            var gain = Mathf.Clamp(feedbackGain, 0f, 0.5f);
            var adjustArrivalLinks = (feedbackStep % 2) == 0;
            if (!TryApplyFeedback(ab, bc, ad, dc, gain, adjustArrivalLinks))
            {
                Restore(stored);
                status = "補正行列を更新できない。";
                RefreshOverlay();
                return false;
            }

            feedbackStep++;
            if (!TryReadPathMatrices(out ab, out bc, out ad, out dc))
            {
                Restore(stored);
                status = "更新後のリンクを読めない。";
                RefreshOverlay();
                return false;
            }

            var after = FrobeniusGap(bc * ab, dc * ad);
            var excessAfter = Mathf.Abs(after - spinGap);
            if (excessAfter > excessBefore + ExcessRiseTolerance)
            {
                Restore(stored);
                status = "この歩はスピン基準から離れるため、静止補正を戻した。";
                RecordCurrent();
                return true;
            }

            status = string.Format(
                CultureInfo.InvariantCulture,
                "歩 {0}: ズレ {1:F4}、スピン基準 {2:F4}",
                feedbackStep,
                Mathf.Sqrt(after),
                Mathf.Sqrt(spinGap));
            RecordCurrent();
            return true;
        }

        [ContextMenu("安定化を8ステップ")]
        public int RunSteps(int count)
        {
            var ran = 0;
            var steps = Mathf.Max(0, count);
            for (var i = 0; i < steps; i++)
            {
                if (!StabilizeStep())
                    break;
                ran++;
            }

            RefreshOverlay();
            return ran;
        }

        public void ClearLog()
        {
            log.Clear();
            logStep = 0;
            feedbackStep = 0;
            initialPlaquetteEnergy = -1f;
            accumulatedLogEnergy = 0f;
            status = "";
            overlayText = "";
        }

        bool TryApplyFeedback(Matrix4x4 ab, Matrix4x4 bc, Matrix4x4 ad, Matrix4x4 dc, float gain, bool adjustArrivalLinks)
        {
            if (!TrySpinProducts(out var pureAbc, out var pureAdc))
                return false;

            if (adjustArrivalLinks)
            {
                if (!TryInvert(ab, out var inverseAb) || !TryInvert(ad, out var inverseAd))
                    return false;
                return TryStoreBlendedEffective(linkBC, nodeB, nodeC, bc, pureAbc * inverseAb, gain)
                    && TryStoreBlendedEffective(linkDC, nodeD, nodeC, dc, pureAdc * inverseAd, gain);
            }

            if (!TryInvert(bc, out var inverseBc) || !TryInvert(dc, out var inverseDc))
                return false;

            return TryStoreBlendedEffective(linkAB, nodeA, nodeB, ab, inverseBc * pureAbc, gain)
                && TryStoreBlendedEffective(linkAD, nodeA, nodeD, ad, inverseDc * pureAdc, gain);
        }

        bool TrySpinProducts(out Matrix4x4 pureAbc, out Matrix4x4 pureAdc)
        {
            var spinAb = UniversalLink.SpinCoupling(nodeA, nodeB);
            var spinBc = UniversalLink.SpinCoupling(nodeB, nodeC);
            var spinAd = UniversalLink.SpinCoupling(nodeA, nodeD);
            var spinDc = UniversalLink.SpinCoupling(nodeD, nodeC);
            pureAbc = spinBc * spinAb;
            pureAdc = spinDc * spinAd;
            return true;
        }

        void AdvanceSpins()
        {
            nodeA.AdvancePhase();
            nodeB.AdvancePhase();
            nodeC.AdvancePhase();
            nodeD.AdvancePhase();
        }

        bool HasCell()
        {
            return nodeA != null
                && nodeB != null
                && nodeC != null
                && nodeD != null
                && linkAB != null
                && linkBC != null
                && linkAD != null
                && linkDC != null
                && linkAB.Connects(nodeA) && linkAB.Connects(nodeB)
                && linkBC.Connects(nodeB) && linkBC.Connects(nodeC)
                && linkAD.Connects(nodeA) && linkAD.Connects(nodeD)
                && linkDC.Connects(nodeD) && linkDC.Connects(nodeC);
        }

        bool TryReadPathMatrices(out Matrix4x4 ab, out Matrix4x4 bc, out Matrix4x4 ad, out Matrix4x4 dc)
        {
            ab = Matrix4x4.identity;
            bc = Matrix4x4.identity;
            ad = Matrix4x4.identity;
            dc = Matrix4x4.identity;
            return linkAB.TryGetTransportMatrix(nodeA, out ab)
                && linkBC.TryGetTransportMatrix(nodeB, out bc)
                && linkAD.TryGetTransportMatrix(nodeA, out ad)
                && linkDC.TryGetTransportMatrix(nodeD, out dc);
        }

        bool TryStoreBlendedEffective(
            UniversalLink link,
            SourceNode from,
            SourceNode to,
            Matrix4x4 currentEffective,
            Matrix4x4 desiredEffective,
            float gain)
        {
            var blended = Lerp(currentEffective, desiredEffective, gain);
            var spin = UniversalLink.SpinCoupling(from, to);
            if (!TryInvert(spin, out var inverseSpin))
                return false;

            var rest = inverseSpin * blended;
            if (!TryNormalizeDeterminant(ref rest))
                return false;

            if (link.EndpointA == from)
            {
                link.SetTransportMatrix(rest);
                return true;
            }

            if (link.EndpointB == from && TryInvert(rest, out var stored))
            {
                link.SetTransportMatrix(stored);
                return true;
            }

            return false;
        }

        Vector4 CommunicationProbe()
        {
            var state = nodeA.State;
            if (state.sqrMagnitude < 1e-8f)
                return new Vector4(1f, 0f, 0f, 0f);
            return state;
        }

        StoredMatrices CaptureStored()
        {
            return new StoredMatrices(
                linkAB.RestMatrix,
                linkBC.RestMatrix,
                linkAD.RestMatrix,
                linkDC.RestMatrix);
        }

        void Restore(StoredMatrices stored)
        {
            linkAB.SetTransportMatrix(stored.Ab);
            linkBC.SetTransportMatrix(stored.Bc);
            linkAD.SetTransportMatrix(stored.Ad);
            linkDC.SetTransportMatrix(stored.Dc);
        }

        void Append(CurvatureResidual residual)
        {
            if (initialPlaquetteEnergy < 0f)
                initialPlaquetteEnergy = residual.PlaquetteEnergy;

            accumulatedLogEnergy += residual.PlaquetteEnergy;
            if (log.Count == LogCapacity)
                log.RemoveAt(0);
            log.Add(residual);
        }

        void RefreshOverlay()
        {
            var culture = CultureInfo.InvariantCulture;
            var latest = Latest;
            var lines = new List<string>();
            if (log.Count == 0)
            {
                lines.Add("結果: ログは空。");
            }
            else
            {
                var currentGap = Mathf.Sqrt(latest.PlaquetteEnergy);
                var spinGap = Mathf.Sqrt(latest.SpinPlaquetteEnergy);
                lines.Add(string.Format(
                    culture,
                    "定常ズレの大きさ: {0:F4}",
                    currentGap));
                lines.Add(string.Format(
                    culture,
                    "スピン基準: {0:F4}    過剰: {1:F4}    直近平均: {2:F4}",
                    spinGap,
                    Mathf.Abs(currentGap - spinGap),
                    RecentMeanGap()));
            }

            lines.Add("棒は経路差の大きさ。黄色い印はスピンが維持する基準。");
            lines.Add(status);

            if (log.Count > 0)
            {
                lines.Add(string.Format(
                    culture,
                    "残差 A→B→C − A→D→C: {0}",
                    FormatVector(latest.Residual)));
                lines.Add(string.Format(
                    culture,
                    "位相 A {0:F3}  B {1:F3}  C {2:F3}  D {3:F3}",
                    nodeA.Phase,
                    nodeB.Phase,
                    nodeC.Phase,
                    nodeD.Phase));
                lines.Add(string.Format(
                    culture,
                    "A→B→C {0}",
                    FormatVector(latest.AlongAbc)));
                lines.Add(string.Format(
                    culture,
                    "A→D→C {0}",
                    FormatVector(latest.AlongAdc)));
                lines.Add(string.Format(
                    culture,
                    "ログ歩 {0}。Aの固有時間 {1}、Cの固有時間 {2}。",
                    latest.LogStep,
                    latest.SourceProperTime,
                    latest.ArrivalProperTime));
            }

            overlayText = string.Join("\n", lines);
        }

        void DrawEnergyChart(Rect rect)
        {
            GUI.Box(rect, GUIContent.none);
            if (log.Count == 0)
                return;

            var maxGap = 1e-8f;
            for (var i = 0; i < log.Count; i++)
            {
                maxGap = Mathf.Max(maxGap, Mathf.Sqrt(log[i].PlaquetteEnergy));
                maxGap = Mathf.Max(maxGap, Mathf.Sqrt(log[i].SpinPlaquetteEnergy));
            }

            var inner = new Rect(rect.x + 6f, rect.y + 6f, rect.width - 12f, rect.height - 12f);
            var slot = inner.width / log.Count;
            var previous = GUI.color;
            for (var i = 0; i < log.Count; i++)
            {
                var gap = Mathf.Sqrt(log[i].PlaquetteEnergy);
                var spinGap = Mathf.Sqrt(log[i].SpinPlaquetteEnergy);
                var height = inner.height * (gap / maxGap);
                var bar = new Rect(
                    inner.x + slot * i + 1f,
                    inner.y + inner.height - height,
                    Mathf.Max(1f, slot - 2f),
                    Mathf.Max(1f, height));
                var age = log.Count == 1 ? 1f : i / (float)(log.Count - 1);
                GUI.color = Color.Lerp(new Color(0.75f, 0.35f, 0.2f), new Color(0.15f, 0.4f, 0.55f), age);
                GUI.DrawTexture(bar, Texture2D.whiteTexture);

                var markHeight = inner.height * (spinGap / maxGap);
                GUI.color = new Color(0.95f, 0.75f, 0.15f);
                GUI.DrawTexture(
                    new Rect(bar.x, inner.y + inner.height - markHeight, bar.width, 2f),
                    Texture2D.whiteTexture);
            }

            GUI.color = previous;
        }

        float RecentMeanGap()
        {
            if (log.Count == 0)
                return 0f;

            var count = Mathf.Min(16, log.Count);
            var sum = 0f;
            for (var i = log.Count - count; i < log.Count; i++)
                sum += Mathf.Sqrt(log[i].PlaquetteEnergy);
            return sum / count;
        }

        void LogSteadyState()
        {
            if (log.Count == 0)
                return;

            var latest = Latest;
            var currentGap = Mathf.Sqrt(latest.PlaquetteEnergy);
            var spinGap = Mathf.Sqrt(latest.SpinPlaquetteEnergy);
            Debug.Log(string.Format(
                CultureInfo.InvariantCulture,
                "[LogSystem] 定常ズレの大きさ {0:F4}、スピン基準 {1:F4}、過剰 {2:F4}、直近平均 {3:F4}",
                currentGap,
                spinGap,
                Mathf.Abs(currentGap - spinGap),
                RecentMeanGap()));
        }

        static Matrix4x4 Lerp(Matrix4x4 from, Matrix4x4 to, float gain)
        {
            var result = from;
            var keep = 1f - gain;
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                    result[row, column] = keep * from[row, column] + gain * to[row, column];
            }

            return result;
        }

        static Matrix4x4 MultiplyScalar(Matrix4x4 matrix, float scalar)
        {
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                    matrix[row, column] *= scalar;
            }

            return matrix;
        }

        static bool TryNormalizeDeterminant(ref Matrix4x4 matrix)
        {
            var determinant = matrix.determinant;
            var absolute = Mathf.Abs(determinant);
            if (absolute < SingularDeterminant || float.IsNaN(determinant) || float.IsInfinity(determinant))
                return false;

            var scale = Mathf.Pow(absolute, -0.25f);
            matrix = MultiplyScalar(matrix, scale);
            return true;
        }

        static bool TryInvert(Matrix4x4 matrix, out Matrix4x4 inverse)
        {
            if (Mathf.Abs(matrix.determinant) < SingularDeterminant)
            {
                inverse = Matrix4x4.identity;
                return false;
            }

            inverse = matrix.inverse;
            return true;
        }

        static float FrobeniusGap(Matrix4x4 left, Matrix4x4 right)
        {
            var sum = 0f;
            for (var row = 0; row < 4; row++)
            {
                for (var column = 0; column < 4; column++)
                {
                    var delta = left[row, column] - right[row, column];
                    sum += delta * delta;
                }
            }

            return sum;
        }

        static string FormatVector(Vector4 value)
        {
            return string.Format(
                CultureInfo.InvariantCulture,
                "({0:F4}, {1:F4}, {2:F4}, {3:F4})",
                value.x,
                value.y,
                value.z,
                value.w);
        }

        readonly struct StoredMatrices
        {
            public StoredMatrices(Matrix4x4 ab, Matrix4x4 bc, Matrix4x4 ad, Matrix4x4 dc)
            {
                Ab = ab;
                Bc = bc;
                Ad = ad;
                Dc = dc;
            }

            public Matrix4x4 Ab { get; }
            public Matrix4x4 Bc { get; }
            public Matrix4x4 Ad { get; }
            public Matrix4x4 Dc { get; }
        }
    }

    static class SampleOverlayLayout
    {
        const float Margin = 12f;
        const float Gap = 10f;

        public static void GetRects(out Rect checkPanel, out Rect graphPanel)
        {
            var bounds = new Rect(
                Margin,
                Margin,
                Mathf.Max(32f, Screen.width - Margin * 2f),
                Mathf.Max(32f, Screen.height - Margin * 2f));

            if (bounds.width >= 760f)
            {
                var checkWidth = Mathf.Clamp(bounds.width * 0.42f, 280f, 460f);
                checkPanel = new Rect(bounds.x, bounds.y, checkWidth, bounds.height);
                graphPanel = new Rect(
                    bounds.x + checkWidth + Gap,
                    bounds.y,
                    bounds.width - checkWidth - Gap,
                    bounds.height);
                return;
            }

                var checkHeight = Mathf.Min(bounds.height * 0.38f, 260f);
                checkHeight = Mathf.Clamp(checkHeight, 32f, Mathf.Max(32f, bounds.height - Gap - 32f));
                checkPanel = new Rect(bounds.x, bounds.y, bounds.width, checkHeight);
                graphPanel = new Rect(
                    bounds.x,
                    bounds.y + checkHeight + Gap,
                    bounds.width,
                    bounds.height - checkHeight - Gap);
        }
    }
}
