# Written Coordination Plan
## AROC (Aliaksandr) ↔ BluPassion (Katja)

**Version:** 1.0  
**Date:** November 08, 2025  
**Validity:** MVP Development Phase (Nov 2025 - Feb 2026)

---

## 1. Why Written Communication?

### Benefits for Both Sides:

**For Aliaksandr:**
- ⏰ Time for precise formulation of technical details
- 📖 Use of translation tools when needed
- 📝 Referenceable documentation of all decisions
- 🧠 Asynchronous processing at own rhythm

**For BluPassion (Katja):**
- 📄 Complete written specifications as work basis
- 🔍 Traceable history of all technical decisions
- ⏱️ No need to coordinate meeting times (efficient time use)
- 📋 Direct integration of specifications into code/docs

**For the Project:**
- ✅ No misunderstandings due to language barrier
- ✅ Matthias can mediate precisely when needed
- ✅ All technical decisions documented
- ✅ Asynchronous collaboration across time zones possible

---

## 2. Communication Channels & Tools

### 2.1 Primary Channel: Email

**Usage:**
- Basic questions & answers
- Status updates
- Notifications about new documents

**Best Practices:**
- ✅ Clear subject line: `[AE.HUB MVP] Topic - Brief Description`
- ✅ Structured emails (numbered points, lists)
- ✅ Concrete questions instead of open discussions
- ✅ Attachments: Diagrams, code snippets, screenshots

**Response Times:**
- Standard: 24h (weekdays)
- Urgent (marked in subject): 4-6h
- Weekend: Best effort, no guarantee

---

### 2.2 Secondary Channel: Shared Documents

**Usage:**
- API specifications (collaborative editing)
- Architecture diagrams
- Open questions lists
- Change log

**Recommended Tools:**
- **Google Docs:** For text documents (API specs, Q&A)
- **Markdown Files (GitHub Gist):** For technical specifications
- **Draw.io / Excalidraw:** For diagrams
- **Notion / Confluence:** If preferred by BluPassion

**Best Practices:**
- ✅ Versioning with date
- ✅ Mark changes with initials (e.g., `[AN 08.11]`)
- ✅ Use comments for discussions
- ✅ Mark "final" version with color

---

### 2.3 Code & Examples: GitHub

**Usage:**
- Share code snippets
- API examples (request/response)
- Bugfixes / patches

**Format:**
- GitHub Gist (for small snippets)
- Pull requests (for larger changes)
- Issues (for bug reports / feature requests)

---

### 2.4 Escalation: Matthias as Coordinator

**When to Involve Matthias:**
- ❗ Unclear/contradictory requirements
- ❗ Technical decisions with business impact
- ❗ Blocking problems after 48h without solution
- ❗ Translation needs for complex topics

**Process:**
1. Aliaksandr/Katja sends email to Matthias (CC: other party)
2. Matthias clarifies bilaterally (email/call)
3. Matthias sends consolidated answer to both

---

## 3. Communication Rhythm

### 3.1 Weekly Status Update (Every Friday)

**From: Aliaksandr**  
**To: Katja, CC: Matthias**  
**Subject:** `[AE.HUB MVP] Weekly Status CW XX/2025 - AROC Connector`

**Template:**
```
Hi Katja,

Here's the weekly status update for the AROC Connector:

## Completed This Week:
- [x] Implemented Feature X
- [x] Fixed Bug Y
- [x] Updated Documentation Z

## In Progress (Next Week):
- [ ] Start Feature A
- [ ] Test Feature B

## Blockers / Need Input:
- Question about API endpoint XYZ (see email from DD.MM)
- Unclear: JSON schema for payload ABC

## Open Questions for You:
1. ...
2. ...

Best regards,
Aliaksandr
```

**From: BluPassion (Katja)**  
**To: Aliaksandr, CC: Matthias**  
**Subject:** `[AE.HUB MVP] Weekly Status CW XX/2025 - AE.HUB Platform`

**Template:** (analogous to above, but for cloud platform side)

---

### 3.2 Ad-hoc Communication

**When:**
- Technical question blocks work
- Unclear specification
- Error/bug discovered
- API change necessary

**Response Time:**
- Standard: 24h
- Urgent (marked): 4-6h

---

## 4. API Coordination Process

### Phase 1: Clarify Initial Questions (CW 45/2025)

**Step 1: BluPassion Sends Questions Document**

Email template:
```
Subject: [AE.HUB MVP] API Clarification Questions - Initial

Hi Aliaksandr,

Attached are our initial questions about the API specification.
We've created a Google Doc so we can work collaboratively:
[Link to Google Doc]

Please review the questions and answer directly in the document.
Matthias can mediate when unclear.

Timeframe: We would appreciate answers by [Date].

Best regards,
Katja
```

**Google Doc Structure:**
```markdown
# API Clarification Questions BluPassion → AROC

## 1. MQTT Interface

### Question 1.1: Topic Structure for Errors
**Katja (08.11.2025):**
Should errors be published on a separate topic,
or in the same status topic?

**Aliaksandr (09.11.2025):**
[Enter answer here]

### Question 1.2: QoS Level
**Katja:**
Is QoS 1 sufficient, or do we need QoS 2 for critical commands?

**Aliaksandr:**
[...]

## 2. WebRTC Interface

...
```

**Step 2: Aliaksandr Answers in Document**

- Answers directly under question
- Mark with date and initials
- If uncertain: "Matthias, please input on..." → @mention

**Step 3: Review & Finalization**

- Both sides review all answers
- Matthias marks final decisions as "✅ FINAL"
- Document becomes "API Contract Document v1.0"

---

### Phase 2: Ongoing API Evolution (During Development)

**Change Request Process:**

1. **Initiate Change:**
   ```
   Subject: [AE.HUB MVP] API Change Request - [Brief Description]

   Hi [Name],

   I propose the following change to the API:

   **What:** [Description]
   **Why:** [Rationale]
   **Impact:** [Affected components]

   **Proposal:**
   [New JSON schema / topic structure / etc.]

   Please provide feedback by [Date].

   Regards, [Name]
   ```

2. **Review:**
   - Other side checks impact
   - Response within 24-48h

3. **Decision:**
   - ✅ Accepted → Both implement
   - ⚠️ Accepted with modifications → Iteration
   - ❌ Rejected → Rationale + alternative

4. **Documentation:**
   - API Contract Document is updated
   - Change log is maintained

---

## 5. Question Templates

### Template 1: Technical Specification Question

```
Subject: [AE.HUB MVP] Question: [Component] - [Brief Description]

Hi [Name],

I have a question about the [component/interface]:

**Context:**
[What am I trying to implement?]

**Question:**
[Specific, concrete question]

**My Assumption:**
[What I understood so far]

**Options (if applicable):**
A) [Option 1]
B) [Option 2]

Do you prefer A or B, or do you have another idea?

Need input by: [Date]

Thanks,
[Name]
```

---

### Template 2: Bug Report

```
Subject: [AE.HUB MVP] Bug Report: [Brief Description]

Hi [Name],

I've discovered an issue:

**Symptom:**
[What happens (or doesn't)?]

**Expected Behavior:**
[What should happen?]

**Steps to Reproduce:**
1. [Step 1]
2. [Step 2]
...

**Environment:**
- Version: [...]
- System: [...]

**Logs/Screenshots:**
[Attachment or link]

**Impact:**
[Blocking me / Nice-to-fix / Critical]

**My Analysis (if available):**
[What I've found out so far]

Regards,
[Name]
```

---

### Template 3: Diagram Request

```
Subject: [AE.HUB MVP] Diagram Request: [What Should Be Visualized]

Hi [Name],

To clarify [topic], a diagram would be helpful.

**Question:**
[What do I want to understand?]

**Desired Representation:**
- Data flow: From A to B to C
- Or: Component overview
- Or: Sequence diagram for workflow X

**Purpose:**
[Why do I need this? What problem does it solve?]

If you have time, a quick whiteboard photo or
Draw.io diagram would be super helpful.

Thanks,
[Name]
```

---

## 6. Document Structure & Versioning

### 6.1 Central Documents

| Document | Owner | Format | Update Frequency |
|----------|-------|--------|------------------|
| **API Contract Document** | Joint | Google Doc / Markdown | As needed (change requests) |
| **Open Questions List** | Joint | Google Doc | Daily during clarification phase |
| **Change Log** | Matthias | Markdown | With every API change |
| **Technical Spec (AROC)** | Aliaksandr | Markdown | Weekly |
| **Technical Spec (Platform)** | BluPassion | Markdown | Weekly |

### 6.2 Versioning Scheme

**API Contract Document:**
- v0.1, v0.2, ... (during clarification)
- v1.0 (final version before development start)
- v1.1, v1.2, ... (minor changes during development)
- v2.0 (breaking changes)

**Naming Convention:**
```
API_Contract_Document_v1_0.md
API_Contract_Document_v1_1_Change_WebRTC_DataChannel.md
```

**Change Note in Document:**
```markdown
## Version History

| Version | Date | Changes | Author |
|---------|------|---------|--------|
| 1.0 | 15.11.2025 | Initial final version | Matthias |
| 1.1 | 22.11.2025 | Added error codes to MQTT | Aliaksandr |
| 1.2 | 29.11.2025 | Changed Data Channel format | Katja |
```

---

## 7. Example: First Week (CW 45/2025)

### Monday, Nov 11, 2025

**09:00 - Katja:**
- Email to Aliaksandr + Matthias
- Subject: `[AE.HUB MVP] API Clarification Questions - Initial`
- Attachment: Google Doc with 15 questions

**17:00 - Aliaksandr:**
- Confirmation: "Thanks, will review this week"

---

### Tuesday, Nov 12, 2025

**14:00 - Aliaksandr:**
- Works on Google Doc
- Answers questions 1-8
- At question 9: "Matthias, please input - concerns business logic"

**18:00 - Matthias:**
- Reads Google Doc
- Clarifies question 9 with Aliaksandr (brief call in German)
- Writes answer in doc

---

### Wednesday, Nov 13, 2025

**10:00 - Katja:**
- Reads Aliaksandr's answers
- New follow-up questions on 3 points (comments in doc)

**16:00 - Aliaksandr:**
- Answers comments

---

### Thursday, Nov 14, 2025

**11:00 - Matthias:**
- Review: All questions clarified?
- Yes → Marks document as "✅ READY FOR FINAL"
- Creates "API_Contract_Document_v1_0.md"
- Email to both: "API finalized, see attachment"

---

### Friday, Nov 15, 2025

**15:00 - Aliaksandr:**
- Weekly status email
- "API clarification completed, starting config file integration"

**16:00 - Katja:**
- Weekly status email
- "API clarification completed, starting backend setup"

---

## 8. Emergency Communication

### Critical Blockers

**Definition:** A problem prevents further work for > 24h

**Process:**
1. Email with `[URGENT]` in subject
2. Brief problem description
3. Matthias in CC
4. Expected response: 4-6h

**Example:**
```
Subject: [AE.HUB MVP] [URGENT] WebRTC Connection Failing

Hi Aliaksandr,

Since this morning I can't establish WebRTC connection anymore.
Error message: "ICE failed, disconnected"

Has anything changed in the Janus configuration?

Currently can't continue work on video integration.

Please quick info.

Regards,
Katja
```

---

## 9. Success Metrics

**How Do We Measure Communication Quality?**

- ✅ **Response Time:** Average < 24h
- ✅ **Clarity:** Less than 2 follow-up questions per initial question
- ✅ **Completion:** All questions answered before deadline
- ✅ **Documentation:** All decisions documented in writing

**Review After Sprint 1:**
- Both sides give feedback on process
- Adjustments if needed

---

## 10. Checklist: Effective Written Communication

**Before I Send an Email:**
- [ ] Subject line clear and specific?
- [ ] Context sufficiently explained?
- [ ] Concrete question asked (not "What do you think about...")?
- [ ] Deadline specified if time-critical?
- [ ] Attachments/links working?
- [ ] CC to Matthias if relevant?

**When Answering:**
- [ ] All questions answered?
- [ ] If unclear: Ask back, don't guess!
- [ ] Answer structured (numbers, lists)?
- [ ] Code/examples where needed?
- [ ] Timeframe stated if I need more info?

---

## 11. Contacts

**Aliaksandr Nazaruk**
- Email: (coordinated via Matthias)
- Response time: 24h (weekdays)
- Languages: Russian (native), English (technical), German (basic)

**BluPassion (Katja)**
- Email: [TBD]
- Response time: 24h (weekdays)

**Matthias Heddinga** (Coordinator)
- Email: matthias.heddinga@aroc-technologies.com
- Availability: Mon-Fri 09:00-18:00 CET
- Role: Mediation, translation, decisions

---

## 12. Appendix: Useful Tools & Resources

### Translation Tools (for Aliaksandr)
- DeepL: https://www.deepl.com (very good for technical German)
- Google Translate: https://translate.google.com
- Linguee: https://www.linguee.com (for technical terms)

### Diagram Tools
- Draw.io: https://app.diagrams.net (free, no registration)
- Excalidraw: https://excalidraw.com (quick sketches)
- Mermaid: https://mermaid.live (for code-generated diagrams)

### Markdown Editors
- StackEdit: https://stackedit.io (online, collaborative)
- GitHub Gist: https://gist.github.com (for code snippets)

---

**End of Coordination Plan**

**Version:** 1.0  
**Date:** November 08, 2025  
**Review:** After Sprint 1 (feedback from both sides)

---

**Good luck with efficient written collaboration!** 🚀
