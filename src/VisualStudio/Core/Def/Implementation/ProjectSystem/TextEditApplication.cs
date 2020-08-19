﻿// Licensed to the .NET Foundation under one or more agreements.
// The .NET Foundation licenses this file to you under the MIT license.
// See the LICENSE file in the project root for more information.

using System.Collections.Generic;
using Microsoft.CodeAnalysis.Editor.Shared.Extensions;
using Microsoft.CodeAnalysis.Editor.Undo;
using Microsoft.CodeAnalysis.Text;
using Microsoft.VisualStudio.Text;

namespace Microsoft.VisualStudio.LanguageServices.Implementation.ProjectSystem
{
    internal static class TextEditApplication
    {
        internal static void UpdateText(SourceText newText, ITextBuffer buffer, EditOptions options)
        {
            using var edit = buffer.CreateEdit(options, reiteratedVersionNumber: null, editTag: null);
            var oldSnapshot = buffer.CurrentSnapshot;
            var oldText = oldSnapshot.AsText();
            var changes = newText.GetTextChanges(oldText);
            if (CodeAnalysis.Workspace.TryGetWorkspace(oldText.Container, out var workspace))
            {
                var undoService = workspace.Services.GetService<ISourceTextUndoService>();
                undoService.BeginUndoTransaction(oldSnapshot);
            }

            foreach (var change in changes)
            {
                edit.Replace(change.Span.Start, change.Span.Length, change.NewText);
            }

            edit.ApplyAndLogExceptions();
        }

        internal static void UpdateText(IEnumerable<TextChange> textChanges, ITextBuffer buffer, EditOptions options)
        {
            using var edit = buffer.CreateEdit(options, reiteratedVersionNumber: null, editTag: null);
            var oldSnapshot = buffer.CurrentSnapshot;
            var oldText = oldSnapshot.AsText();
            if (CodeAnalysis.Workspace.TryGetWorkspace(oldText.Container, out var workspace))
            {
                var undoService = workspace.Services.GetService<ISourceTextUndoService>();
                undoService.BeginUndoTransaction(oldSnapshot);
            }

            foreach (var change in textChanges)
            {
                edit.Replace(change.Span.Start, change.Span.Length, change.NewText);
            }

            edit.ApplyAndLogExceptions();
        }
    }
}
